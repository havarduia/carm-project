#!/usr/bin/env python3
"""ROS 2 Humble ArUco detector using RealSense ROS topics.

This node is intended as an aruco_ros-style replacement for easy_handeye2.
It subscribes to:
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)

And publishes:
- TF transform(s) for detected marker(s)
- PoseStamped for selected marker on /aruco_single/pose
- Annotated debug image on /aruco_single/result
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
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

PNP_FLAGS: Dict[str, int] = {
    "ITERATIVE": cv2.SOLVEPNP_ITERATIVE,
    "IPPE_SQUARE": cv2.SOLVEPNP_IPPE_SQUARE,
    "SQPNP": cv2.SOLVEPNP_SQPNP,
}


class ArucoRealsenseTfNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_realsense_tf_node")

        self._declare_parameters()
        self._load_parameters()
        self._validate_configuration()

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.image_subscriber = self.create_subscription(
            Image,
            self.color_image_topic,
            self._image_callback,
            10,
        )
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10,
        )

        self.window_name = "aruco_realsense_debug"
        self.last_tvec_by_id: Dict[int, np.ndarray] = {}

        self.camera_matrix: Optional[np.ndarray] = None
        self.distortion_coefficients: Optional[np.ndarray] = None
        self._camera_info_received = False

        self.dictionary = self._load_dictionary(self.aruco_dictionary_name)
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector_parameters.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
            if self.use_subpixel_refinement
            else cv2.aruco.CORNER_REFINE_NONE
        )

        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            "Started ArUco node. Waiting for camera_info on '%s', images on '%s'"
            % (self.camera_info_topic, self.color_image_topic)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("marker_id", -1)
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("marker_frame_prefix", "aruco_marker_")
        self.declare_parameter("single_marker_frame_id", "aruco_marker")

        self.declare_parameter("color_image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")

        self.declare_parameter("pose_topic", "/aruco_single/pose")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/aruco_single/result")
        self.declare_parameter("show_debug_window", False)

        self.declare_parameter("use_subpixel_refinement", True)
        self.declare_parameter("pnp_method", "IPPE_SQUARE")
        self.declare_parameter("use_lm_refinement", True)
        self.declare_parameter("max_reprojection_error_px", 3.0)
        self.declare_parameter("pose_smoothing_alpha", 0.0)

    def _load_parameters(self) -> None:
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.aruco_dictionary_name = str(self.get_parameter("aruco_dictionary").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.marker_frame_prefix = str(self.get_parameter("marker_frame_prefix").value)
        self.single_marker_frame_id = str(self.get_parameter("single_marker_frame_id").value)

        self.color_image_topic = str(self.get_parameter("color_image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)

        self.use_subpixel_refinement = bool(self.get_parameter("use_subpixel_refinement").value)
        self.pnp_method = str(self.get_parameter("pnp_method").value)
        self.use_lm_refinement = bool(self.get_parameter("use_lm_refinement").value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").value)
        self.pose_smoothing_alpha = float(self.get_parameter("pose_smoothing_alpha").value)

    def _validate_configuration(self) -> None:
        if self.marker_size <= 0.0:
            raise ValueError("marker_size must be > 0")
        if self.aruco_dictionary_name not in ARUCO_DICTIONARIES:
            raise ValueError(f"Invalid aruco_dictionary '{self.aruco_dictionary_name}'")
        if self.pnp_method not in PNP_FLAGS:
            raise ValueError(f"Invalid pnp_method '{self.pnp_method}'")
        if not 0.0 <= self.pose_smoothing_alpha < 1.0:
            raise ValueError("pose_smoothing_alpha must be in [0.0, 1.0)")

    def _load_dictionary(self, dictionary_name: str) -> cv2.aruco.Dictionary:
        return cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary_name])

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.distortion_coefficients = np.array(msg.d, dtype=np.float64)
        if len(self.distortion_coefficients.shape) == 1:
            self.distortion_coefficients = self.distortion_coefficients.reshape((-1, 1))

        if not self._camera_info_received:
            self._camera_info_received = True
            self.get_logger().info("Received camera intrinsics from '%s'" % self.camera_info_topic)

    def _on_parameter_update(self, params):
        for param in params:
            if param.name == "marker_id":
                self.marker_id = int(param.value)
            elif param.name == "marker_size":
                v = float(param.value)
                if v <= 0.0:
                    return SetParametersResult(successful=False, reason="marker_size must be > 0")
                self.marker_size = v
            elif param.name == "aruco_dictionary":
                name = str(param.value)
                if name not in ARUCO_DICTIONARIES:
                    return SetParametersResult(successful=False, reason=f"Invalid aruco_dictionary '{name}'")
                self.aruco_dictionary_name = name
                self.dictionary = self._load_dictionary(name)
            elif param.name == "publish_debug_image":
                self.publish_debug_image = bool(param.value)
            elif param.name == "show_debug_window":
                self.show_debug_window = bool(param.value)
            elif param.name == "use_subpixel_refinement":
                self.use_subpixel_refinement = bool(param.value)
                self.detector_parameters.cornerRefinementMethod = (
                    cv2.aruco.CORNER_REFINE_SUBPIX
                    if self.use_subpixel_refinement
                    else cv2.aruco.CORNER_REFINE_NONE
                )
            elif param.name == "use_lm_refinement":
                self.use_lm_refinement = bool(param.value)
            elif param.name == "pnp_method":
                name = str(param.value)
                if name not in PNP_FLAGS:
                    return SetParametersResult(successful=False, reason=f"Invalid pnp_method '{name}'")
                self.pnp_method = name
            elif param.name == "max_reprojection_error_px":
                self.max_reprojection_error_px = float(param.value)
            elif param.name == "pose_smoothing_alpha":
                a = float(param.value)
                if not 0.0 <= a < 1.0:
                    return SetParametersResult(successful=False, reason="pose_smoothing_alpha must be in [0.0, 1.0)")
                self.pose_smoothing_alpha = a
        return SetParametersResult(successful=True)

    def _image_callback(self, msg: Image) -> None:
        if self.camera_matrix is None or self.distortion_coefficients is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image with cv_bridge: {exc}")
            return

        self._process_frame(image, msg)

    def _process_frame(self, image: np.ndarray, image_msg: Image) -> None:
        corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.detector_parameters)
        debug_image = image.copy() if (self.publish_debug_image or self.show_debug_window) else None

        if ids is not None and len(ids) > 0:
            if debug_image is not None:
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

            for idx, detected_id in enumerate(ids.flatten()):
                marker_id = int(detected_id)
                if self.marker_id >= 0 and marker_id != self.marker_id:
                    continue

                rvec, tvec, reproj_error = self._estimate_pose(corners[idx])
                if rvec is None or tvec is None:
                    continue
                if self.max_reprojection_error_px > 0.0 and reproj_error > self.max_reprojection_error_px:
                    continue

                tvec = self._smooth_translation(marker_id, tvec)
                stamp = image_msg.header.stamp

                transform = self._build_transform(marker_id, rvec, tvec, stamp)
                self.tf_broadcaster.sendTransform(transform)

                if self.marker_id >= 0:
                    self.pose_publisher.publish(self._build_pose_msg(rvec, tvec, stamp))

                if debug_image is not None:
                    cv2.drawFrameAxes(
                        debug_image,
                        self.camera_matrix,
                        self.distortion_coefficients,
                        rvec,
                        tvec,
                        self.marker_size * 0.5,
                    )
                    self._draw_marker_label(debug_image, corners[idx], marker_id, tvec, reproj_error)

        if debug_image is not None:
            self._publish_debug_image(debug_image, image_msg.header)
            if self.show_debug_window:
                cv2.imshow(self.window_name, debug_image)
                cv2.waitKey(1)

    def _estimate_pose(self, marker_corners: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float]:
        half_size = self.marker_size * 0.5
        object_points = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )
        image_points = marker_corners.reshape(4, 2).astype(np.float64)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.distortion_coefficients,
            flags=PNP_FLAGS[self.pnp_method],
        )
        if not success:
            return None, None, 1e9

        if self.use_lm_refinement:
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points,
                image_points,
                self.camera_matrix,
                self.distortion_coefficients,
                rvec,
                tvec,
            )

        projected_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.distortion_coefficients,
        )
        reproj_error = float(np.mean(np.linalg.norm(projected_points.reshape(-1, 2) - image_points, axis=1)))
        return rvec, tvec, reproj_error

    def _smooth_translation(self, marker_id: int, tvec: np.ndarray) -> np.ndarray:
        if self.pose_smoothing_alpha <= 0.0:
            return tvec
        previous = self.last_tvec_by_id.get(marker_id)
        if previous is None:
            self.last_tvec_by_id[marker_id] = tvec
            return tvec
        smoothed = self.pose_smoothing_alpha * previous + (1.0 - self.pose_smoothing_alpha) * tvec
        self.last_tvec_by_id[marker_id] = smoothed
        return smoothed

    def _publish_debug_image(self, image: np.ndarray, header) -> None:
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header = header
        self.debug_image_publisher.publish(msg)

    def _draw_marker_label(
        self,
        image: np.ndarray,
        marker_corners: np.ndarray,
        marker_id: int,
        tvec: np.ndarray,
        reproj_error: float,
    ) -> None:
        text_point = marker_corners[0][0].astype(int)
        distance = float(np.linalg.norm(tvec.reshape(3)))
        label = f"id={marker_id} z={float(tvec[2]):.3f}m d={distance:.3f}m err={reproj_error:.2f}px"
        cv2.putText(
            image,
            label,
            (int(text_point[0]), int(text_point[1]) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def _build_transform(self, marker_id: int, rvec: np.ndarray, tvec: np.ndarray, stamp) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = (
            self.single_marker_frame_id if self.marker_id >= 0 else f"{self.marker_frame_prefix}{marker_id}"
        )

        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(rotation_matrix)

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def _build_pose_msg(self, rvec: np.ndarray, tvec: np.ndarray, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.camera_frame
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(rotation_matrix)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

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
