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
    offset_z = -13

    rclpy.init(args=args)

    node = YoloSnapshotNode()
    
    # Initialize xArm
    arm = XArmAPI('192.168.1.225')
    time.sleep(0.5)
    arm.set_tcp_maxacc(1000)
    speed = 100
    moveto = move(speed=speed, arm=arm)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_gripper_position(800, wait=True)
    # Go home
    moveto.home()

    # We spin repeatedly looking for target_position
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        
        if node.target_position is not None:
            # target_position captured when 's' is pressed

            x, y, z = node.target_position
            if z < 5:
                print("Invalid target position detected, skipping...")
                node.target_position = None
                continue
            print(f"Captured target in main: {x}, {y}, {z}")
            
            # TODO: add xarm API code here to move the arm using x, y, z
            # e.g.: move_arm(x, y, z)
            arm.set_position(*[x+offset_x, y, z+offset_z, 180, 0, 0], wait=True)
            arm.set_gripper_position(0, wait=True)
            arm.set_position(*[x+offset_x, y, z+offset_z+40, 180, 0, 0], wait=True)
            moveto.place()
            moveto.home()

            # Reset it so it waits for another 's' press
            node.target_position = None

    

    arm.disconnect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()