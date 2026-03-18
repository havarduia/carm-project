# carm-project

This repository contains an xArm pick workflow that combines ROS2, an Intel RealSense camera, Roboflow-hosted YOLO inference, and a calibrated static TF between the camera and robot base. The primary runtime entrypoints are `Xarm_code/main.py` for the full robot workflow and `Xarm_code/yolo_model.py` for detection and 3D target estimation only.

## Repository layout

- `Xarm_code/main.py` — full application entrypoint that initializes the xArm, waits for a detected target, then performs pick/place motion.
- `Xarm_code/yolo_model.py` — ROS2 node that subscribes to RealSense image/depth/camera info topics, runs YOLO inference, converts detections to 3D points, and transforms them into the robot base frame.
- `Xarm_code/movement.py` — helper motion routines used by `main.py`.
- `Xarm_code/aruco_realsense_tf_node.py` — calibration/setup helper, not the main runtime workflow.
- `calibration/generate_aruco.py` — helper script for generating ArUco boards/markers.

## Main workflow

`Xarm_code/main.py` is the main runtime script. It:

1. Starts the `YoloSnapshotNode` from `yolo_model.py`.
2. Connects to the xArm at `192.168.1.225`.
3. Sends the arm to its home position and opens the gripper.
4. Waits until a valid target position is captured.
5. Moves the robot to the detected point with hard-coded offsets:
   - `offset_x = 10`
   - `offset_z = -13`
6. Closes the gripper, lifts the object, executes the place routine, and returns home.

The detected point is expected in millimeters in the `link_base` frame after TF transformation. Targets with `z < 5` are rejected as invalid before motion is executed.

## Detection workflow

`Xarm_code/yolo_model.py` provides the perception pipeline used by `main.py`.

### Subscribed topics

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`

### What it does

- Displays the incoming color feed in an OpenCV window.
- Waits for keyboard input:
  - Press `s` to run detection on the current frame.
  - Press `q` to shut down the ROS2 node.
- Sends the captured image to Roboflow through `InferenceHTTPClient`.
- Filters detections using `CONF_THRESHOLD = 0.70`.
- Selects the highest-confidence detection.
- Uses the aligned depth image plus camera intrinsics to compute the 3D point in `camera_color_optical_frame`.
- Transforms the point into `link_base` using TF.
- Stores the transformed point in `self.target_position` for `main.py` to consume.
- Draws the detection and transformed robot-frame coordinates in an OpenCV debug window.

### YOLO / inference configuration

The current implementation in `Xarm_code/yolo_model.py` uses:

- Roboflow API endpoint: `https://serverless.roboflow.com`
- Workspace: `carm-yitb6`
- Workflow ID: `small-object-detection-sahi-2`
- Confidence threshold: `0.70`

## Required launch order

Launch the system in this order.

### 1) Start the RealSense ROS2 driver

The runtime expects the RealSense ROS topics used in `yolo_model.py`, including aligned depth.

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  enable_rgbd:=true
```

### 2) Publish the static transform before launching detection or robot control

`yolo_model.py` transforms detected 3D points from `camera_color_optical_frame` into `link_base`, so this transform must already be available.

```bash
ros2 run tf2_ros static_transform_publisher \
  0.092638 -0.123698 0.363872 \
  -0.834954 0.443900 -0.145063 0.291138 \
  link_base camera_color_optical_frame
```

### 3) Confirm the required camera topics exist

```bash
ros2 topic list | grep /camera/camera
```

Expected topics include:

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`

### 4) Run detection only (optional)

Use this if you want to test YOLO detection and the TF-transformed target position without moving the robot.

```bash
python3 Xarm_code/yolo_model.py
```

When the camera window appears, press `s` to run inference on the current frame.

### 5) Run the full xArm workflow

Use this for the complete detect-and-pick flow.

```bash
python3 Xarm_code/main.py
```

`main.py` will:

- create the YOLO snapshot node,
- connect to the arm,
- home the arm,
- wait for a detection trigger from the OpenCV window,
- pick at the transformed target position,
- place the object,
- return home.

## Runtime notes

- `main.py` currently connects to the robot using the hard-coded IP `192.168.1.225`.
- `yolo_model.py` expects a working internet connection so it can call the hosted Roboflow inference API.
- The camera-to-robot calibration is assumed to be represented by the static TF above.
- `aruco_realsense_tf_node.py` is intended for calibration/setup and is not the primary runtime documented here.
- Both scripts rely on OpenCV windows and keyboard input, so they should be run in an environment with GUI access.

## Dependencies

The Python scripts import the following libraries/packages:

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `image_geometry`
- `tf2_ros`
- `tf2_geometry_msgs`
- `opencv-python`
- `numpy`
- `inference_sdk`
- `xarm`

Example ROS dependency install for `cv_bridge`:

```bash
sudo apt install ros-humble-cv-bridge
```

Depending on your ROS2 installation, you may also need the TF and image geometry packages from your ROS distro plus Python packages for `inference_sdk` and the xArm SDK.
