# carm-project
Project with the University of Agder, more info coming soon.

## ROS2 Humble ArUco + RealSense TF node

A ready-to-run ROS2 Python node is available at:

- `Xarm_code/aruco_realsense_tf_node.py`

It uses:
- `pyrealsense2` for camera stream + intrinsics,
- OpenCV ArUco for marker detection,
- TF2 broadcaster for publishing marker pose transforms.

### Key configurable ROS parameters

- `marker_id` (`int`, default `-1`):
  - `-1` publishes TF for all detected markers,
  - any non-negative ID publishes only that marker.
- `marker_size` (`double`, default `0.05`): marker side length in meters.
- `aruco_dictionary` (`string`, default `DICT_4X4_50`): e.g. `DICT_6X6_250`.
- `camera_frame` (`string`, default `camera_color_optical_frame`): TF parent frame.
- `marker_frame_prefix` (`string`, default `aruco_marker_`): child frame prefix.
- `width`/`height`/`fps`: RealSense color stream settings.
- `publish_rate_hz`: detection loop rate.
- `publish_debug_image` (`bool`, default `true`): publish annotated detection image on a ROS topic.
- `debug_image_topic` (`string`, default `/aruco/debug_image`): debug image topic name.
- `show_debug_window` (`bool`, default `false`): show OpenCV window with overlays (for local GUI debugging).


### Debug visualization

You can inspect detections in two ways:

- ROS image topic (recommended):
  - enable `publish_debug_image:=true` and view `/aruco/debug_image` in RViz2 or `rqt_image_view`.
- Local OpenCV window:
  - set `show_debug_window:=true` to pop up a live annotated view.

### Example run

```bash
ros2 run <your_package_name> aruco_realsense_tf_node --ros-args \
  -p marker_id:=23 \
  -p marker_size:=0.04 \
  -p aruco_dictionary:=DICT_6X6_250 \
  -p publish_debug_image:=true
```

> Tip: this script is currently provided as a standalone file in this repository.
> To run with `ros2 run`, place it in a ROS2 Python package and register it as a console script.
