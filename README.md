# carm-project
Project with the University of Agder, more info coming soon.

## ROS2 Humble ArUco + RealSense TF node

A ready-to-run ROS2 Python node is available at:

- `Xarm_code/aruco_realsense_tf_node.py`

It uses:
- `pyrealsense2` for camera stream + intrinsics,
- OpenCV ArUco for marker detection,
- TF2 broadcaster for publishing marker pose transforms,
- `PoseStamped` output compatible with `aruco_ros`-style easy_handeye2 workflows.

### `aruco_ros` replacement behavior for easy_handeye2

When `marker_id` is set to a specific marker (e.g. `23`), the node publishes:

- TF: `camera_frame -> single_marker_frame_id` (default `aruco_marker`)
- Pose topic: `/aruco_single/pose` (configurable via `pose_topic`)
- Debug image: `/aruco_single/result` (configurable via `debug_image_topic`)

This mirrors the common `aruco_ros` + `easy_handeye2` usage pattern.

### Key configurable ROS parameters

- Marker setup
  - `marker_id` (`int`, default `-1`): `-1` = all markers, non-negative = track single marker.
  - `marker_size` (`double`, default `0.05`): marker side length in meters.
  - `aruco_dictionary` (`string`, default `DICT_4X4_50`): e.g. `DICT_6X6_250`.
- Frames / topics
  - `camera_frame` (`string`, default `camera_color_optical_frame`)
  - `marker_frame_prefix` (`string`, default `aruco_marker_`) for multi-marker mode.
  - `single_marker_frame_id` (`string`, default `aruco_marker`) for single-marker mode.
  - `pose_topic` (`string`, default `/aruco_single/pose`)
  - `debug_image_topic` (`string`, default `/aruco_single/result`)
- Camera / timing
  - `width`, `height`, `fps`
  - `publish_rate_hz`
- Debug
  - `publish_debug_image` (`bool`, default `true`)
  - `show_debug_window` (`bool`, default `false`)
- Accuracy tuning
  - `use_subpixel_refinement` (`bool`, default `true`)
  - `pnp_method` (`string`, default `IPPE_SQUARE`, choices: `IPPE_SQUARE`, `ITERATIVE`, `SQPNP`)
  - `use_lm_refinement` (`bool`, default `true`)
  - `max_reprojection_error_px` (`double`, default `3.0`)
  - `pose_smoothing_alpha` (`double`, default `0.0`, range `[0.0, 1.0)`)

### Why calibration accuracy is better now

- Subpixel corner refinement improves corner localization.
- PnP is solved with selectable methods (default `IPPE_SQUARE` is typically strong for planar square markers).
- Levenberg-Marquardt refinement (`solvePnPRefineLM`) further improves pose.
- Pose estimates can be filtered by reprojection error threshold.
- Optional temporal smoothing reduces jitter in calibration capture.

### Example run (easy_handeye2-style single marker)

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

> Tip: this script is currently provided as a standalone file in this repository.
> To run with `ros2 run`, place it in a ROS2 Python package and register it as a console script.
