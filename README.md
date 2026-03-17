# carm-project
Project with the University of Agder, more info coming soon.

## ROS2 Humble ArUco + RealSense TF node

A ready-to-run ROS2 Python node is available at:

- `Xarm_code/aruco_realsense_tf_node.py`

This implementation now uses the **ROS2 RealSense driver topics** (not `pyrealsense2`) and is suitable as an `aruco_ros`-style replacement for easy_handeye2.

## Topic-based camera interface (RealSense ROS driver)

Subscribe inputs:
- `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)

Publish outputs:
- TF transforms for marker pose(s)
- `/aruco_single/pose` (`geometry_msgs/msg/PoseStamped`, configurable)
- `/aruco_single/result` (`sensor_msgs/msg/Image`, configurable)

## `aruco_ros` replacement behavior for easy_handeye2

When `marker_id` is set to a specific marker (e.g. `23`), the node publishes:

- TF: `camera_frame -> single_marker_frame_id` (default `aruco_marker`)
- Pose topic: `/aruco_single/pose`
- Debug image: `/aruco_single/result`

This mirrors the common single-marker setup used by hand-eye calibration tools.

## Key configurable ROS parameters

- Marker setup
  - `marker_id` (`int`, default `-1`): `-1` = all markers, non-negative = single marker.
  - `marker_size` (`double`, default `0.05`): marker side length in meters.
  - `aruco_dictionary` (`string`, default `DICT_4X4_50`): e.g. `DICT_6X6_250`.
- Frames / topics
  - `camera_frame` (`string`, default `camera_color_optical_frame`)
  - `marker_frame_prefix` (`string`, default `aruco_marker_`) for multi-marker mode
  - `single_marker_frame_id` (`string`, default `aruco_marker`) for single-marker mode
  - `color_image_topic` (`string`, default `/camera/color/image_raw`)
  - `camera_info_topic` (`string`, default `/camera/color/camera_info`)
  - `pose_topic` (`string`, default `/aruco_single/pose`)
  - `debug_image_topic` (`string`, default `/aruco_single/result`)
- Debug
  - `publish_debug_image` (`bool`, default `true`)
  - `show_debug_window` (`bool`, default `false`)
- Accuracy tuning
  - `use_subpixel_refinement` (`bool`, default `true`)
  - `pnp_method` (`string`, default `IPPE_SQUARE`, choices: `IPPE_SQUARE`, `ITERATIVE`, `SQPNP`)
  - `use_lm_refinement` (`bool`, default `true`)
  - `max_reprojection_error_px` (`double`, default `3.0`)
  - `pose_smoothing_alpha` (`double`, default `0.0`, range `[0.0, 1.0)`)

## Validation flow

### 1) Start RealSense ROS2 driver

```bash
ros2 launch realsense2_camera rs_launch.py color_width:=1920 color_height:=1080 color_fps:=30
```

### 2) Confirm camera topics exist

```bash
ros2 topic list | grep camera
```

Expected includes:
- `/camera/color/image_raw`
- `/camera/color/camera_info`

### 3) Run detector node

```bash
ros2 run <your_package_name> aruco_realsense_tf_node --ros-args \
  -p marker_id:=23 \
  -p marker_size:=0.04 \
  -p aruco_dictionary:=DICT_6X6_250 \
  -p single_marker_frame_id:=aruco_marker \
  -p pose_topic:=/aruco_single/pose \
  -p debug_image_topic:=/aruco_single/result \
  -p use_subpixel_refinement:=true \
  -p pnp_method:=IPPE_SQUARE \
  -p use_lm_refinement:=true
```

### 4) (Optional) View debug image

```bash
ros2 topic echo /aruco_single/result
```

Use `rqt_image_view` or RViz2 for visualization.

## Dependencies

- `ros-humble-cv-bridge`
- `opencv-python`
- `numpy`
- `rclpy`
- `tf2_ros`

Install cv_bridge if missing:

```bash
sudo apt install ros-humble-cv-bridge
```

> Tip: this script is currently provided as a standalone file in this repository.
> To run with `ros2 run`, place it in a ROS2 Python package and register it as a console script.
