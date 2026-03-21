import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs

import cv2
import numpy as np
import tempfile

from inference_sdk import InferenceHTTPClient


CONF_THRESHOLD = 0.70


class YoloSnapshotNode(Node):

    def __init__(self, target_class=None):
        super().__init__('yolo_snapshot_node')
        
        # Can be set to "capacitor", "resistor", "transformer", or None for any
        self.target_class = target_class

        self.bridge = CvBridge()
        self.frame = None
        self.depth = None
        self.camera_info_ready = False

        self.camera_model = PinholeCameraModel()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            10
        )

        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key="LhkBeIDhei8ywxS1RaCw"
        )

        self.target_positions = None

        self.get_logger().info("Press 's' to detect, 'q' to quit")

    def info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_ready = True

    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def image_callback(self, msg):

        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        cv2.imshow("camera", self.frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.run_detection()

        if key == ord('q'):
            rclpy.shutdown()

    def run_detection(self):

        if self.frame is None or self.depth is None:
            print("Waiting for camera data...")
            return

        if not self.camera_info_ready:
            print("Waiting for camera info...")
            return

        if self.camera_model.P is None:
            print("Camera model not initialized yet...")
            return

        frame = self.frame.copy()

        result = self.client.run_workflow(
            workspace_name="carm-yitb6",
            workflow_id="small-object-detection-sahi-4",
            images={"image": frame},
            use_cache=True
        )

        preds = result[0]['predictions']['predictions']

        # Filter detections by confidence
        preds = [p for p in preds if p['confidence'] >= CONF_THRESHOLD]
        
        # Filter detections by target class
        if self.target_class:
            preds = [p for p in preds if p['class'].lower() == self.target_class.lower()]

        if len(preds) == 0:
            print(f"No valid detections above threshold or matching target class '{self.target_class}'")
            return

        h, w = self.depth.shape
        valid_targets = []

        for p in preds:
            u = int(p['x'])
            v = int(p['y'])

            # clamp to avoid border crash
            u = np.clip(u, 2, w-3)
            v = np.clip(v, 2, h-3)

            window = self.depth[v-2:v+3, u-2:u+3]
            Z = np.median(window) / 1000.0

            if Z == 0:
                print("Invalid depth for one target, skipping...")
                continue

            ray = self.camera_model.projectPixelTo3dRay((u, v))

            X = ray[0] * Z
            Y = ray[1] * Z

            point = PointStamped()
            point.header.frame_id = "camera_color_optical_frame"
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = float(X)
            point.point.y = float(Y)
            point.point.z = float(Z)

            try:
                robot_point = self.tf_buffer.transform(
                    point,
                    "link_base",
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                rx = robot_point.point.x
                ry = robot_point.point.y
                rz = robot_point.point.z

                valid_targets.append((rx * 1000.0, ry * 1000.0, rz * 1000.0))

            except Exception as e:
                print("TF transform failed:", e)
                continue

        # Draw detections
        for p in preds:

            x = int(p['x'])
            y = int(p['y'])
            w = int(p['width'])
            h = int(p['height'])

            x1 = int(x - w/2)
            y1 = int(y - h/2)
            x2 = int(x + w/2)
            y2 = int(y + h/2)

            label = f"{p['class']} {p['confidence']:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        cv2.putText(frame, "Multiple detections shown", (30,40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        cv2.imshow("detection", frame)
        cv2.waitKey(100)  # Force OpenCV to render the window before movement

        if valid_targets:
            self.target_positions = valid_targets
            return valid_targets
        return None


def main(args=None):

    rclpy.init(args=args)

    # Set the target class here: "capacitor", "resistor", "transformer", or None for any
    node = YoloSnapshotNode(target_class="capacitor")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()