# carm-project

This repository contains an xArm pick workflow combining ROS2, an Intel RealSense camera, and Roboflow YOLO inference.

## Prerequisites

Before running `main.py`, you need to run the following commands in separate terminals:

### 1. Start the RealSense Camera
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true enable_rgbd:=true
```

### 2. Publish the Static Transform
```bash
ros2 run tf2_ros static_transform_publisher --x 0.092638 --y -0.123698 --z 0.363872 --qx -0.834954 --qy 0.443900 --qz -0.145063 --qw 0.291138 --frame-id link_base --child-frame-id camera_color_optical_frame
```

---

### 3. Run the Main Script
Once the above are running, you can execute the main script:
```bash
python3 main/main.py
```

## Repository Structure & Files

- `main/main.py`: The main runtime script. Initializes the robot and runs the pick workflow.
- `detection_model/yolo_model.py`: ROS2 node that processes RealSense camera feeds, runs YOLO inference through Roboflow, computes 3D points, and applies the static TF to output coordinates in the robot base frame.
- `helpers/movement.py`: Contains helper functions for xArm motion control, used by the main script for pick and place routines.
- `calibration/aruco_realsense_tf_node.py`: A utility script for calculating camera-to-robot transforms using ArUco markers.
- `calibration/generate_aruco.py`: Generates printable ArUco markers/boards for calibration.

## Requirements

- **ROS2** (Humble/Foxy or preferred distro)
- **librealsense2** and **realsense-ros**
- **xArm-Python-SDK** (for robot control)
- **Roboflow / inference sdk** (for YOLO model integration)
- **OpenCV** (for image processing and marker generation)