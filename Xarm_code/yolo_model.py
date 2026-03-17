import tempfile
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from image_geometry import PinholeCameraModel
from inference_sdk import InferenceHTTPClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from visualization_msgs.msg import Marker


CONF_THRESHOLD = 0.80


@dataclass
class DetectionResult:
    label: str
    confidence: float
    pixel_u: int
    pixel_v: int
    camera_x: float
    camera_y: float
    camera_z: float
    robot_x: float
    robot_y: float
    robot_z: float
    annotated_frame: np.ndarray


class YoloSnapshotNode(Node):
    """Detects a component from camera topics and exposes robot-frame coordinates."""

    def __init__(self):
        super().__init__('yolo_snapshot_node')

        self.bridge = CvBridge()
        self.frame: Optional[np.ndarray] = None
        self.depth: Optional[np.ndarray] = None
        self.camera_model = PinholeCameraModel()
        self.camera_info_ready = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10,
        )
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            10,
        )

        self.point_pub = self.create_publisher(PointStamped, '/component_detection/target_point', 10)
        self.marker_pub = self.create_publisher(Marker, '/component_detection/target_marker', 10)
        self.preview_pub = self.create_publisher(Image, '/component_detection/preview', 10)

        self.client = InferenceHTTPClient(
            api_url='https://serverless.roboflow.com',
            api_key='LhkBeIDhei8ywxS1RaCw',
        )

        self.last_result: Optional[DetectionResult] = None
        self.get_logger().info("YOLO node ready. Press 's' in the camera window to detect, 'q' to quit.")

    def info_callback(self, msg: CameraInfo):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_ready = True

    def depth_callback(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def image_callback(self, msg: Image):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('camera', self.frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.detect_latest_frame()
        if key == ord('q'):
            rclpy.shutdown()

    def detect_latest_frame(self) -> Optional[DetectionResult]:
        if self.frame is None or self.depth is None or not self.camera_info_ready:
            self.get_logger().warning('Waiting for image, depth and camera info before detection.')
            return None

        frame = self.frame.copy()
        temp = tempfile.NamedTemporaryFile(suffix='.png', delete=False)
        cv2.imwrite(temp.name, frame)

        result = self.client.run_workflow(
            workspace_name='carm-yitb6',
            workflow_id='small-object-detection-sahi-2',
            images={'image': temp.name},
            use_cache=True,
        )

        preds = result[0]['predictions']['predictions']
        preds = [p for p in preds if p['confidence'] >= CONF_THRESHOLD]

        if not preds:
            self.get_logger().info('No detections above threshold.')
            return None

        best = max(preds, key=lambda x: x['confidence'])

        u = int(best['x'])
        v = int(best['y'])

        h, w = self.depth.shape
        u = int(np.clip(u, 2, w - 3))
        v = int(np.clip(v, 2, h - 3))

        window = self.depth[v - 2:v + 3, u - 2:u + 3]
        z = float(np.median(window) / 1000.0)
        if z <= 0.0:
            self.get_logger().warning('Invalid depth sample at detected location.')
            return None

        ray = self.camera_model.projectPixelTo3dRay((u, v))
        x = float(ray[0] * z)
        y = float(ray[1] * z)

        point = PointStamped()
        point.header.frame_id = 'camera_color_optical_frame'
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z

        try:
            robot_point = self.tf_buffer.transform(
                point,
                'link_base',
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'TF transform failed: {exc}')
            return None

        for p in preds:
            px = int(p['x'])
            py = int(p['y'])
            pw = int(p['width'])
            ph = int(p['height'])
            x1 = int(px - pw / 2)
            y1 = int(py - ph / 2)
            x2 = int(px + pw / 2)
            y2 = int(py + ph / 2)
            label = f"{p['class']} {p['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        result_obj = DetectionResult(
            label=best['class'],
            confidence=float(best['confidence']),
            pixel_u=u,
            pixel_v=v,
            camera_x=x,
            camera_y=y,
            camera_z=z,
            robot_x=float(robot_point.point.x),
            robot_y=float(robot_point.point.y),
            robot_z=float(robot_point.point.z),
            annotated_frame=frame,
        )

        self.last_result = result_obj
        self.publish_result(result_obj)

        cv2.putText(
            frame,
            f"Robot xyz: {result_obj.robot_x:.3f}, {result_obj.robot_y:.3f}, {result_obj.robot_z:.3f}",
            (25, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        cv2.imshow('detection', frame)

        self.get_logger().info(
            f"Detected {result_obj.label} at robot xyz "
            f"({result_obj.robot_x:.3f}, {result_obj.robot_y:.3f}, {result_obj.robot_z:.3f})"
        )
        return result_obj

    def publish_result(self, result: DetectionResult) -> None:
        point_msg = PointStamped()
        point_msg.header.frame_id = 'link_base'
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = result.robot_x
        point_msg.point.y = result.robot_y
        point_msg.point.z = result.robot_z
        self.point_pub.publish(point_msg)

        marker = Marker()
        marker.header = point_msg.header
        marker.ns = 'component_detection'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = result.robot_x
        marker.pose.position.y = result.robot_y
        marker.pose.position.z = result.robot_z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        target_tf = TransformStamped()
        target_tf.header = point_msg.header
        target_tf.child_frame_id = 'detected_component'
        target_tf.transform.translation.x = result.robot_x
        target_tf.transform.translation.y = result.robot_y
        target_tf.transform.translation.z = result.robot_z
        target_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(target_tf)

        preview_msg = self.bridge.cv2_to_imgmsg(result.annotated_frame, encoding='bgr8')
        preview_msg.header = point_msg.header
        self.preview_pub.publish(preview_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSnapshotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
