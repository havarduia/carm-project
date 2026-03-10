#!/usr/bin/env python3
"""ROS 2 Humble node for ArUco detection with Intel RealSense.

This node:
- Captures color images from a RealSense camera with pyrealsense2.
- Reads camera intrinsics directly from the stream profile.
- Detects ArUco markers with OpenCV.
- Publishes the selected marker pose as TF.

Parameters make marker ID, marker dictionary, and marker size easy to change.
"""

from __future__ import annotations

from typing import Dict

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


ARUCO_DICTIONARIES: Dict[str, int] = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoRealsenseTfNode(Node):
    """Detect ArUco markers with RealSense input and publish TF transforms."""

    def __init__(self) -> None:
        super().__init__("aruco_realsense_tf_node")

        self.declare_parameter("marker_id", -1)
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("marker_frame_prefix", "aruco_marker_")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/aruco/debug_image")
        self.declare_parameter("show_debug_window", False)

        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.aruco_dictionary_name = str(self.get_parameter("aruco_dictionary").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.marker_frame_prefix = str(self.get_parameter("marker_frame_prefix").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)

        self._validate_configuration()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, 10)
        self.window_name = "aruco_realsense_debug"

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.profile = self.pipeline.start(self.config)

        color_stream_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_stream_profile.get_intrinsics()

        self.camera_matrix = np.array(
            [[intrinsics.fx, 0.0, intrinsics.ppx], [0.0, intrinsics.fy, intrinsics.ppy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        self.distortion_coefficients = np.array(intrinsics.coeffs, dtype=np.float64)

        self.dictionary = self._load_dictionary(self.aruco_dictionary_name)
        self.detector_parameters = cv2.aruco.DetectorParameters()

        interval = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(interval, self._process_frame)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            "Started ArUco RealSense TF node with marker_id=%d, marker_size=%.3f m, dictionary=%s"
            % (self.marker_id, self.marker_size, self.aruco_dictionary_name)
        )

    def _validate_configuration(self) -> None:
        if self.marker_size <= 0.0:
            raise ValueError("marker_size must be > 0")
        if self.aruco_dictionary_name not in ARUCO_DICTIONARIES:
            raise ValueError(
                f"Invalid aruco_dictionary '{self.aruco_dictionary_name}'. "
                f"Options: {', '.join(sorted(ARUCO_DICTIONARIES.keys()))}"
            )

    def _load_dictionary(self, dictionary_name: str) -> cv2.aruco.Dictionary:
        return cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary_name])

    def _on_parameter_update(self, params):
        for param in params:
            if param.name == "marker_id":
                self.marker_id = int(param.value)
            elif param.name == "marker_size":
                new_marker_size = float(param.value)
                if new_marker_size <= 0.0:
                    return SetParametersResult(successful=False, reason="marker_size must be > 0")
                self.marker_size = new_marker_size
            elif param.name == "aruco_dictionary":
                name = str(param.value)
                if name not in ARUCO_DICTIONARIES:
                    return SetParametersResult(
                        successful=False,
                        reason=f"Invalid aruco_dictionary '{name}'",
                    )
                self.aruco_dictionary_name = name
                self.dictionary = self._load_dictionary(name)
            elif param.name == "publish_debug_image":
                self.publish_debug_image = bool(param.value)
            elif param.name == "show_debug_window":
                self.show_debug_window = bool(param.value)
        return SetParametersResult(successful=True)

    def _process_frame(self) -> None:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        image = np.asanyarray(color_frame.get_data())
        corners, ids, _ = cv2.aruco.detectMarkers(
            image,
            self.dictionary,
            parameters=self.detector_parameters,
        )
        debug_image = image.copy() if (self.publish_debug_image or self.show_debug_window) else None

        if ids is not None and len(ids) > 0:
            if debug_image is not None:
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
            ids_flat = ids.flatten()
            rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.distortion_coefficients,
            )

            for idx, detected_id in enumerate(ids_flat):
                if self.marker_id >= 0 and detected_id != self.marker_id:
                    continue

                transform = self._build_transform(int(detected_id), rvecs[idx], tvecs[idx])
                self.tf_broadcaster.sendTransform(transform)

                if debug_image is not None:
                    cv2.drawFrameAxes(
                        debug_image,
                        self.camera_matrix,
                        self.distortion_coefficients,
                        rvecs[idx],
                        tvecs[idx],
                        self.marker_size * 0.5,
                    )
                    self._draw_marker_label(debug_image, corners[idx], int(detected_id), tvecs[idx])

        if debug_image is not None:
            self._publish_debug_image(debug_image)
            if self.show_debug_window:
                cv2.imshow(self.window_name, debug_image)
                cv2.waitKey(1)

    def _publish_debug_image(self, image: np.ndarray) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = image.shape[1] * image.shape[2]
        msg.data = image.tobytes()
        self.debug_image_publisher.publish(msg)

    def _draw_marker_label(self, image: np.ndarray, marker_corners: np.ndarray, marker_id: int, tvec: np.ndarray) -> None:
        text_point = marker_corners[0][0].astype(int)
        distance = float(np.linalg.norm(tvec[0]))
        label = f"id={marker_id} z={tvec[0][2]:.2f}m d={distance:.2f}m"
        cv2.putText(
            image,
            label,
            (int(text_point[0]), int(text_point[1]) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def _build_transform(self, marker_id: int, rvec: np.ndarray, tvec: np.ndarray) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = f"{self.marker_frame_prefix}{marker_id}"

        transform.transform.translation.x = float(tvec[0][0])
        transform.transform.translation.y = float(tvec[0][1])
        transform.transform.translation.z = float(tvec[0][2])

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(rotation_matrix)

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        return transform


    @staticmethod
    def _rotation_matrix_to_quaternion(rotation_matrix: np.ndarray):
        m00, m01, m02 = rotation_matrix[0]
        m10, m11, m12 = rotation_matrix[1]
        m20, m21, m22 = rotation_matrix[2]

        trace = m00 + m11 + m22
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (m21 - m12) * s
            qy = (m02 - m20) * s
            qz = (m10 - m01) * s
        elif m00 > m11 and m00 > m22:
            s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
            qw = (m21 - m12) / s
            qx = 0.25 * s
            qy = (m01 + m10) / s
            qz = (m02 + m20) / s
        elif m11 > m22:
            s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
            qw = (m02 - m20) / s
            qx = (m01 + m10) / s
            qy = 0.25 * s
            qz = (m12 + m21) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
            qw = (m10 - m01) / s
            qx = (m02 + m20) / s
            qy = (m12 + m21) / s
            qz = 0.25 * s

        return float(qx), float(qy), float(qz), float(qw)

    def destroy_node(self) -> bool:
        self.get_logger().info("Stopping RealSense pipeline.")
        self.pipeline.stop()
        if self.show_debug_window:
            cv2.destroyWindow(self.window_name)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArucoRealsenseTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
