import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.qos import DurabilityPolicy, QoSProfile

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs

import atexit
import collections
import cv2
import math
import numpy as np
import os
import select
import sys
import termios
import tty

from inference_sdk import InferenceHTTPClient


CONF_THRESHOLD = 0.60

# Half-width in px of the median window sampled around a detection centre. Kept
# small and central so it reads the top face of the component, not the table
# around it - a window wider than the part biases the grasp downwards.
DEPTH_HALF_WINDOW = 2

# RealSense reports 0 for pixels it could not measure. Require this many real
# samples out of the window before trusting the median.
MIN_DEPTH_SAMPLES = 5

# Plausible camera-to-table distance in metres. Anything outside is a bad read,
# not a component. Widen if the camera is remounted further away.
DEPTH_RANGE_M = (0.15, 1.5)

# SAHI slices the frame, so one component near a tile seam can be reported
# twice. Targets closer together than this (mm) are the same object.
DUPLICATE_RADIUS_MM = 8.0

# Annotated snapshot, for an RViz2 Image display.
DETECTION_IMAGE_TOPIC = '/yolo/detection_image'

# One box per detection, for an RViz2 MarkerArray display.
DETECTION_BOX_TOPIC = '/yolo/detection_boxes'

# A box thinner than this (m) is invisible edge-on in RViz2. Flat components
# read as a near-zero depth extent, so give every side a floor.
MIN_BOX_SIZE_M = 0.005


def cbreak_stdin():
    """Deliver single keypresses without Enter, and restore the tty on exit.

    cbreak, not raw, so Ctrl-C still interrupts. No-op when stdin is not a
    terminal (piped input, launched from a launch file).
    """
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    atexit.register(termios.tcsetattr, fd, termios.TCSADRAIN, old)


def read_key():
    """One pending keypress from stdin, or None if nothing is waiting."""
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    return sys.stdin.read(1)


def deproject(camera_model, u, v, depth_m):
    """Pixel + depth -> point in the camera optical frame, in metres.

    projectPixelTo3dRay returns a *unit* vector, so scaling it by the depth puts
    the point at range `depth_m` instead of at depth `depth_m`. Rescaling by
    ray[2] fixes that; without it X and Y shrink by up to ~10% at the frame edge.
    """
    ray = camera_model.projectPixelTo3dRay((u, v))
    scale = depth_m / ray[2]
    return ray[0] * scale, ray[1] * scale, depth_m


def bbox_points(camera_model, depth, p):
    """Every usable depth pixel inside a detection box, as camera-frame metres.

    Zeros ("no reading") and anything outside the working range fall out with
    the range check, so a box overhanging the table edge contributes nothing
    rather than a wall of bogus points.
    """
    h, w = depth.shape
    x1 = max(0, int(p['x'] - p['width'] / 2))
    x2 = min(w, int(p['x'] + p['width'] / 2) + 1)
    y1 = max(0, int(p['y'] - p['height'] / 2))
    y2 = min(h, int(p['y'] + p['height'] / 2) + 1)

    points = []
    for v in range(y1, y2):
        for u in range(x1, x2):
            Z = float(depth[v, u]) / 1000.0
            if DEPTH_RANGE_M[0] <= Z <= DEPTH_RANGE_M[1]:
                points.append(deproject(camera_model, u, v, Z))
    return points


def bbox_extent(points):
    """Axis-aligned (centre, size) in metres around deprojected points, or None."""
    if not points:
        return None
    lo = [min(c) for c in zip(*points)]
    hi = [max(c) for c in zip(*points)]
    centre = [(a + b) / 2.0 for a, b in zip(lo, hi)]
    size = [max(b - a, MIN_BOX_SIZE_M) for a, b in zip(lo, hi)]
    return centre, size


class YoloSnapshotNode(Node):

    def __init__(self, target_class=None):
        super().__init__('yolo_snapshot_node')
        
        # Can be set to "capacitor", "resistor", "transformer", or None for any
        self.target_class = target_class

        self.bridge = CvBridge()
        self.frame = None
        self.frame_stamp = None
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
            api_key=os.environ["ROBOFLOW_API_KEY"],
        )

        self.image_pub = self.create_publisher(Image, DETECTION_IMAGE_TOPIC, 1)

        # Snapshots are published once per keypress, so latch them: an RViz2
        # opened after the detection still gets the last boxes.
        self.box_pub = self.create_publisher(
            MarkerArray,
            DETECTION_BOX_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.target_positions = None

        cbreak_stdin()
        self.create_timer(0.1, self.poll_key)

        self.get_logger().info(
            f"Press 's' in this terminal to detect, 'q' to quit. "
            f"Results are published on {DETECTION_IMAGE_TOPIC} "
            f"and {DETECTION_BOX_TOPIC}"
        )

    def poll_key(self):
        key = read_key()
        if key == 's':
            self.run_detection()
        elif key == 'q' and rclpy.ok():
            rclpy.shutdown()

    def publish_frame(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.frame_id = "camera_color_optical_frame"
        if self.frame_stamp is not None:
            msg.header.stamp = self.frame_stamp
        self.image_pub.publish(msg)

    def publish_boxes(self, preds):
        """One hitbox per detection, in the camera optical frame.

        Left in the camera frame on purpose - RViz2 walks the same TF chain to
        link_base that the grasp points go through, so no transform here.
        Visualisation only; MoveIt never sees these.
        """
        header = Header(frame_id="camera_color_optical_frame")
        if self.frame_stamp is not None:
            header.stamp = self.frame_stamp

        # Clears last snapshot's boxes, so a removed component does not linger.
        markers = [Marker(action=Marker.DELETEALL)]

        for i, p in enumerate(preds):
            extent = bbox_extent(bbox_points(self.camera_model, self.depth, p))
            if extent is None:
                continue
            centre, size = extent

            m = Marker(header=header, ns='detections', id=i, type=Marker.CUBE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = centre
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = size
            m.color.g, m.color.a = 1.0, 0.5
            markers.append(m)

        self.box_pub.publish(MarkerArray(markers=markers))

    def info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_ready = True

    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def image_callback(self, msg):

        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # TF is looked up at the moment the frame was captured, not at detection time.
        self.frame_stamp = msg.header.stamp

    def _sample_depth(self, u, v):
        """Median depth in metres around pixel (u, v), or None if untrustworthy."""
        h, w = self.depth.shape
        cu = int(np.clip(round(u), DEPTH_HALF_WINDOW, w - 1 - DEPTH_HALF_WINDOW))
        cv_ = int(np.clip(round(v), DEPTH_HALF_WINDOW, h - 1 - DEPTH_HALF_WINDOW))

        window = self.depth[
            cv_ - DEPTH_HALF_WINDOW:cv_ + DEPTH_HALF_WINDOW + 1,
            cu - DEPTH_HALF_WINDOW:cu + DEPTH_HALF_WINDOW + 1,
        ]

        # Drop the 0s first - they are "no reading", and averaging them in
        # pulls the median towards the camera and the grasp into the table.
        measured = window[window > 0]
        if measured.size < MIN_DEPTH_SAMPLES:
            print(f"Only {measured.size} valid depth pixels at ({cu}, {cv_}), skipping...")
            return None

        Z = float(np.median(measured)) / 1000.0
        if not DEPTH_RANGE_M[0] <= Z <= DEPTH_RANGE_M[1]:
            print(f"Depth {Z:.3f} m outside {DEPTH_RANGE_M} m working range, skipping...")
            return None
        return Z

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

        # Every class the model reported, so you can see what it actually
        # produces and give each one a bin in main.py's PLACE_BINS.
        counts = collections.Counter(p['class'] for p in preds)
        print("Detected: " + (", ".join(f"{n}x {c}" for c, n in counts.most_common()) or "nothing"))

        # Filter by confidence and (optionally) target class
        preds = [
            p for p in preds
            if p['confidence'] >= CONF_THRESHOLD
            and (not self.target_class or p['class'].lower() == self.target_class.lower())
        ]

        if len(preds) == 0:
            print(f"No valid detections above threshold or matching target class '{self.target_class}'")
            self.publish_frame(frame)
            self.publish_boxes([])
            return

        if self.depth.shape[:2] != frame.shape[:2]:
            print(
                f"Depth {self.depth.shape[:2]} does not match colour {frame.shape[:2]}. "
                "Relaunch the RealSense driver with align_depth.enable:=true"
            )
            return

        valid_targets = []

        for p in preds:
            u = float(p['x'])
            v = float(p['y'])

            Z = self._sample_depth(u, v)
            if Z is None:
                continue

            X, Y, Z = deproject(self.camera_model, u, v, Z)

            point = PointStamped()
            point.header.frame_id = "camera_color_optical_frame"
            point.header.stamp = self.frame_stamp
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

                target = (rx * 1000.0, ry * 1000.0, rz * 1000.0, p['class'])

                if any(math.dist(target[:3], t[:3]) < DUPLICATE_RADIUS_MM for t in valid_targets):
                    print(f"Duplicate detection of '{p['class']}' at {target[:3]}, skipping...")
                    continue

                valid_targets.append(target)

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

        self.publish_frame(frame)
        self.publish_boxes(preds)

        if valid_targets:
            self.target_positions = valid_targets
