import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
import tempfile
from inference_sdk import InferenceHTTPClient
from yolo_model import YoloSnapshotNode
from movement import move
import time
from xarm.wrapper import XArmAPI

def main(args=None):    

    offset_x = 10
    offset_z = 50

    # Set the component type to find: "capacitor", "resistor", "transformer", or None for any
    target_component = "transformer"
    rclpy.init(args=args)
    node = YoloSnapshotNode(target_class=target_component)
    
    # Initialize xArm
    arm = XArmAPI('192.168.1.225')
    time.sleep(0.5)
    arm.set_tcp_maxacc(1000)
    speed = 500
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    moveto = move(speed=speed, arm=arm)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_gripper_position(0, wait=True)
    # Go home
    moveto.home()

    current_place_x = 70  # Starting x coordinate for placement
    step_size = 1       # Amount to increase x each cycle

    # We spin repeatedly looking for target_positions
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        
        if node.target_positions:
            # target_positions captured when 's' is pressed
            valid_positions = []
            for pos in node.target_positions:
                if pos[2] >= 5:
                    valid_positions.append(pos)
                else:
                    print(f"Invalid target position ({pos[0]}, {pos[1]}, {pos[2]}) detected, skipping...")

            # Get current arm position to use as the starting point for shortest path
            code, current_pos = arm.get_position()
            if code != 0:
                current_pos = [130, 0, 150]  # Fallback to home coordinates
            
            curr_x, curr_y = current_pos[0], current_pos[1]
            path = []
            
            # Nearest neighbor algorithm for shortest path calculation
            while valid_positions:
                nearest_idx = 0
                min_dist = float('inf')
                for i, pos in enumerate(valid_positions):
                    dist = ((pos[0] + offset_x) - curr_x)**2 + (pos[1] - curr_y)**2
                    if dist < min_dist:
                        min_dist = dist
                        nearest_idx = i
                
                next_pos = valid_positions.pop(nearest_idx)
                path.append(next_pos)
                curr_x, curr_y = next_pos[0] + offset_x, next_pos[1]

            for pos in path:
                x, y, z = pos
                print(f"Captured target in main: {x}, {y}, {z}")
                
                # Move above the component to point at it
                arm.set_position(*[x+offset_x, y, z+offset_z, 180, 0, 0], wait=True)
                time.sleep(0.5)  # Pause to show we are pointing at the component
            
            # Go home after processing all items
            moveto.home()

            # Reset so it waits for another 's' press
            node.target_positions = []

    arm.disconnect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()