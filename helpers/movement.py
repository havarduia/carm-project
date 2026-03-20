import math
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class MoveArm():
    """
    Standardised movements using MoveIt 2 API and Action Clients
    """
    
    def __init__(self, node_name="moveit_planner", group_name="xarm6"):
        # We assume rclpy.init() has been called in main.py
        self.node = rclpy.create_node(node_name)
        
        # Initialize MoveItPy for planning
        self.moveit_py_instance = MoveItPy(node_name=node_name)
        self.arm_group = self.moveit_py_instance.get_planning_component(group_name)
        
        # Action client to execute trajectories
        self.execute_client = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        self.execute_client.wait_for_server(timeout_sec=5.0)

        # Action client for Gripper
        self.gripper_client = ActionClient(self.node, GripperCommand, '/xarm_gripper/gripper_action')
       
    def get_pose(self, x, y, z, roll=180.0, pitch=0.0, yaw=0.0):
        """
        Creates a PoseStamped from coordinates in mm and angles in degrees
        """
        pose = PoseStamped()
        pose.header.frame_id = "link_base"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        
        # Convert mm to meters
        pose.pose.position.x = x / 1000.0
        pose.pose.position.y = y / 1000.0
        pose.pose.position.z = z / 1000.0
        
        # Convert Roll, Pitch, Yaw from degrees to radians, then to Quaternion
        r = math.radians(roll)
        p = math.radians(pitch)
        y_rad = math.radians(yaw)
        
        cy = math.cos(y_rad * 0.5)
        sy = math.sin(y_rad * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)
        
        pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        return pose

    def _plan_and_execute(self, target_pose):
        """
        Calculates a plan using MoveItPy and sends it to the Action Server to run.
        """
        self.arm_group.set_start_state_to_current_state()
        # "link_tcp" is typically the xarm end effector link
        self.arm_group.set_goal_state(pose_stamped_msg=target_pose, pose_link="link_tcp")
        
        plan_result = self.arm_group.plan()
        
        if plan_result and plan_result.trajectory:
            self.node.get_logger().info("Trajectory planned! Sending to execution action client...")
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = plan_result.trajectory
            
            future = self.execute_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.node.get_logger().error("Execution goal rejected.")
                return False
                
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, res_future)
            self.node.get_logger().info("Move finished.")
            return True
        else:
            self.node.get_logger().error("Could not find a valid plan.")
            return False

    def move_to(self, x, y, z):
        pose = self.get_pose(x, y, z)
        return self._plan_and_execute(pose)

    def home(self):
        """
        Goes home to the default coordinate using MoveIt.
        """
        self.node.get_logger().info("Going home...")
        pose = self.get_pose(130, 0, 150)
        self._plan_and_execute(pose)
        
    def place(self, x, z):
        """
        Places the object in the bin.
        """
        self.node.get_logger().info("Placing object...")
        pose = self.get_pose(x, 220.6, z)
        self._plan_and_execute(pose)
        
    def set_gripper(self, pos):
        """
        Sends GripperCommand to the action server. 
        Note: xarm uses 0-850 pos units usually. 
        """
        if not self.gripper_client.server_is_ready():
            self.node.get_logger().warn("Gripper action server not available, mapping out setting gripper.")
            return
            
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(pos)
        goal_msg.command.max_effort = -1.0
        
        self.node.get_logger().info(f"Setting gripper to {pos}...")
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future)

