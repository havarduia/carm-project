# carm-project

This repository contains an xArm pick-and-place workflow that integrates ROS2, an Intel RealSense camera, Roboflow-hosted YOLO inference, and MoveIt 2 for robot motion planning and execution. The system detects objects using YOLO, transforms the 2D coordinates to 3D using RealSense depth data, applies a static TF to find the object's position in the robot's base frame, and then commands the xArm to execute the pick.

## Repository layout

- `main/main.py` — The primary runtime script. Sets up the object detection node and commands the robot to perform the pick-and-place workflow using MoveIt.
- `detection_model/yolo_model.py` — A ROS2 node that processes the RealSense camera feeds (RGB and aligned depth), queries the Roboflow YOLO endpoint, computes 3D points, and applies the static TF to output target coordinates in the robot base frame (`link_base`).
- `helpers/movement.py` — Contains helper classes and functions wrapping MoveIt 2 Action Clients (`ExecuteTrajectory`) and `GripperCommand` to comfortably control the xArm motion and end-effector.
- `calibration/aruco_realsense_tf_node.py` — A utility ROS2 node for calculating the camera-to-robot base transform using ArUco markers.
- `calibration/generate_aruco.py` — Generates printable ArUco markers and boards used for the camera calibration routine.

## Prerequisites & Dependencies

To run this workflow, you need the following installed in your ROS2 environment (tested on Humble/Foxy):

- **ROS2** 
- **librealsense2** and **realsense-ros**
- **xArm ROS2 packages** (specifically `xarm_moveit_config`)
- **MoveIt 2**
- **Roboflow Inference SDK** (`inference_sdk`)
- **OpenCV** Python package (`opencv-python`)
- **cv_bridge** (e.g., `ros-humble-cv-bridge`)

## Required Launch Order

You **must** launch the required ROS2 drivers, static transforms, and the MoveIt controller before running the main workflow. Ensure you run each of the following commands in **separate terminals** sourcing your ROS2 workspace.

### 1) Start the RealSense ROS2 driver

The application relies on aligned depth and RGB topics from the camera.

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true enable_rgbd:=true
```

### 2) Publish the static transform

The static transform maps the camera optical frame to the robot base. This enables the 3D projection of the YOLO detections into a coordinate system the robot understands.

```bash
ros2 run tf2_ros static_transform_publisher --x 0.092638 --y -0.123698 --z 0.363872 --qx -0.834954 --qy 0.443900 --qz -0.145063 --qw 0.291138 --frame-id link_base --child-frame-id camera_color_optical_frame
```

### 3) Launch the MoveIt 2 Configuration

This starts the MoveIt controller to manage motion planning and execution for the xArm. Replace `192.168.1.xxx` with your robot's actual IP address.

```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.xxx add_gripper:=true
```

### 4) Run the full xArm workflow

Once the camera is publishing, the TF is registered, and MoveIt is ready to accept trajectory commands, you can start the main pick-and-place routine.

```bash
python3 main/main.py
```

*Note: The script currently defaults to looking for a `capacitor` (or another manually configured target class inside `main.py`). The camera stream will display in an OpenCV window. By default, pressing `s` on the OpenCV window often triggers the detection snapshot.*