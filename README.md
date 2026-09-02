# carm-project

This repository contains an xArm pick-and-place workflow that integrates ROS2, an Intel RealSense camera, Roboflow-hosted YOLO inference, and MoveIt 2 for robot motion planning and execution. The system detects objects using YOLO, transforms the 2D coordinates to 3D using RealSense depth data, applies a static TF to find the object's position in the robot's base frame, and then commands the xArm to execute the pick.

## Repository layout

- `main/main.py` — The primary runtime script. Sets up the object detection node and commands the robot to perform the pick-and-place workflow using MoveIt.
- `detection_model/yolo_model.py` — A ROS2 node that processes the RealSense camera feeds (RGB and aligned depth), queries the Roboflow YOLO endpoint, computes 3D points, and applies the static TF to output target coordinates in the robot base frame (`link_base`).
- `helpers/movement.py` — Contains helper classes and functions wrapping MoveIt 2 Action Clients (`ExecuteTrajectory`) and `GripperCommand` to comfortably control the xArm motion and end-effector.
- `calibration/aruco_realsense_tf_node.py` — A utility ROS2 node for calculating the camera-to-robot base transform using ArUco markers.
- `calibration/generate_aruco.py` — Generates printable ArUco markers and boards used for the camera calibration routine.

## Prerequisites & Dependencies

To run this workflow, you need the following installed in your ROS2 environment (tested on Humble):

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

The application relies on aligned depth and RGB topics from the camera. `publish_tf:=false` is required — otherwise the driver's own camera TF tree collides with the static transform in step 2 below and breaks the `link_base` → `camera_color_optical_frame` lookup.

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true enable_rgbd:=true publish_tf:=false
```

### 2) Publish the camera-to-base transform

This maps the camera optical frame to the robot base, which is what lets the YOLO
detections be projected into a coordinate system the robot understands. It is published
straight from the saved hand-eye calibration:

```bash
ros2 launch easy_handeye2 publish.launch.py name:=carm_eob
```

If the camera is moved or re-mounted this transform is wrong and the calibration must be
redone — see [Camera calibration (hand-eye)](#camera-calibration-hand-eye) below.

<details>
<summary>Equivalent raw command, if you'd rather not depend on easy_handeye2 at runtime</summary>

From the calibration of 2026-09-02. Regenerate these from
`~/.ros2/easy_handeye2/calibrations/carm_eob.calib` after any recalibration:

```bash
ros2 run tf2_ros static_transform_publisher --x 0.099766 --y -0.121559 --z 0.353031 --qx -0.840483 --qy 0.426063 --qz -0.160939 --qw 0.293526 --frame-id link_base --child-frame-id camera_color_optical_frame
```
</details>

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

### Optional: run without connected hardware

If you do not have a RealSense camera or xArm connected, you can run a mock/demo cycle:

```bash
python3 main/main.py --no-hardware
```

This mode avoids camera subscriptions and robot controllers by using synthetic detections and mocked arm/gripper commands.

*Note: The script currently defaults to looking for a `capacitor` (or another manually configured target class inside `main.py`). The camera stream will display in an OpenCV window. By default, pressing `s` on the OpenCV window often triggers the detection snapshot.*

## Camera calibration (hand-eye)

The camera sits on a fixed mount next to the arm, so this is an **eye-on-base**
calibration: the camera is stationary and the ArUco marker is bolted to the gripper.
The arm waves the marker around, `easy_handeye2` correlates `link_base -> link_eef`
against `camera_color_optical_frame -> aruco_marker`, and solves for
`link_base -> camera_color_optical_frame`.

`calibration/aruco_realsense_tf_node.py` plays the role of `aruco_ros` here, so
that package is not needed.

### One-time setup

[easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) is checked out
and built in `~/carm_ws`:

```bash
cd ~/carm_ws/src && git clone https://github.com/marcoesposito1988/easy_handeye2
cd ~/carm_ws && colcon build --packages-select easy_handeye2_msgs easy_handeye2
```

It needs `transforms3d >= 0.4`. The apt package (`python3-transforms3d`, 0.3.1) uses
`np.float`, which NumPy 1.26 removed, so the node dies on import. Install a newer one
for the *system* interpreter:

```bash
/usr/bin/python3 -m pip install --user -U 'transforms3d>=0.4'
```

> Run every command below with the system Python, **not** this repo's `.venv`.
> `cv_bridge`, `rclpy` and `tf2_ros` are apt-installed into `/usr/lib/python3/dist-packages`
> and are not visible from the venv. `deactivate` first if the venv is active.

### Marker

A single ArUco marker, `DICT_6X6_1000` id **398**. Mount it rigidly and *flat* on the
gripper — any wobble or tilt relative to `link_eef` goes straight into the calibration
error. Marker size is the side of the **black square only**, excluding the white quiet
zone, in metres.

### Procedure

Each step in its own terminal, sourcing `/opt/ros/humble/setup.bash` and
`~/carm_ws/install/setup.bash`.

**1) RealSense driver** — same as the normal launch order:

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true enable_rgbd:=true publish_tf:=false
```

**2) Robot driver / MoveIt** — supplies the `link_base -> link_eef` TF:

```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.xxx add_gripper:=true
```

> Do **not** start the static transform from step 2 of the launch order above.
> `calibrate.launch.py` publishes its own placeholder `link_base -> camera_color_optical_frame`,
> and two publishers fighting over the same TF edge produce garbage samples.

**3) Marker detector:**

```bash
python3 calibration/aruco_realsense_tf_node.py --ros-args \
  -p marker_id:=398 \
  -p marker_size:=0.115 \
  -p aruco_dictionary:=DICT_6X6_1000 \
  -p camera_frame:=camera_color_optical_frame \
  -p single_marker_frame_id:=aruco_marker \
  -p show_debug_window:=true
```

Check that it actually sees the marker before continuing:

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker
```

**4) Calibrator:**

```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  name:=carm_eob \
  robot_base_frame:=link_base \
  robot_effector_frame:=link_eef \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=aruco_marker
```

**5) Sample.** Jog the arm (freedrive or MoveIt) to a pose where the camera sees the
marker, then hit *Take sample* in the rqt window. Repeat for **at least 15 poses**.
What matters:

- **Rotate a lot** between poses — large orientation changes are what make the
  solve well-conditioned. Pure translation adds nothing.
- Keep the marker close to the camera and filling a good chunk of the frame.
- Vary distance and cover the corners of the image, not just the centre.
- Skip any pose where the detector's reprojection error looks high or the axes in the
  debug window jitter.

**6) Compute and save.** *Compute* then *Save*, which writes
`~/.ros2/easy_handeye2/calibrations/carm_eob.calib`.

**7) Use it.** Replace step 2 of the launch order with:

```bash
ros2 launch easy_handeye2 publish.launch.py name:=carm_eob
```

Sanity-check it: put an object at a known spot, run the pick workflow, and confirm the
arm goes where it should. If it is off by a consistent scale factor, `marker_size` is
wrong.
