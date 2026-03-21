import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import GripperCommand
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive


class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm_node")

        self.get_logger().info("Waiting for services and action servers...")

        # Action clients
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.gripper_client = ActionClient(self, GripperCommand, "/xarm_gripper/gripper_action")

        # Cartesian service
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")

        # Planning scene publisher
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self.execute_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.cartesian_client.wait_for_service()

        self.get_logger().info("MoveArm ready.")

        # Add pole
        self.add_pole()
        time.sleep(1)
        self.add_camera_ball()
        time.sleep(1)

    # ==============================
    # Add collision pole
    # ==============================
    def add_pole(self):
        co = CollisionObject()
        co.id = "camera_pole"
        co.header.frame_id = "link_base"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER

        height = 0.78
        radius = 0.02

        primitive.dimensions = [height, radius]

        pose = Pose()
        pose.position.x = 0.053
        pose.position.y = -0.175
        pose.position.z = height / 2.0
        pose.orientation.w = 1.0

        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        self.scene_pub.publish(scene)

        self.get_logger().info("Pole added.")

    def add_camera_ball(self):
        self.get_logger().info("Adding camera collision sphere...")

        co = CollisionObject()
        co.id = "camera_ball"
        co.header.frame_id = "link_base"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE

        radius = 0.06  # 6 cm (adjust if needed)
        primitive.dimensions = [radius]

        pose = Pose()
        pose.position.x = 0.111
        pose.position.y = -0.1504
        pose.position.z = 0.35

        pose.orientation.w = 1.0

        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        self.scene_pub.publish(scene)

        self.get_logger().info("Camera sphere added.")

    # ==============================
    # Pose (fixed downward)
    # ==============================
    def create_pose(self, x, y, z, roll=180.0, pitch=0.0, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "link_base"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x / 1000.0
        pose.pose.position.y = y / 1000.0
        pose.pose.position.z = z / 1000.0

        r = math.radians(roll)
        p = math.radians(pitch)
        y_ = math.radians(yaw)

        cy = math.cos(y_ * 0.5)
        sy = math.sin(y_ * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

        return pose

    # ==============================
    # Speed scaling
    # ==============================
    def scale_trajectory_speed(self, traj, scale):
        for point in traj.joint_trajectory.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            t_scaled = t / scale

            sec = int(t_scaled)
            nanosec = int((t_scaled - sec) * 1e9)
            nanosec = max(0, min(nanosec, 999999999))

            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec

            if point.velocities:
                point.velocities = [v * scale for v in point.velocities]
            if point.accelerations:
                point.accelerations = [a * scale for a in point.accelerations]

        return traj

    # ==============================
    # Cartesian move (FIXED)
    # ==============================
    def cartesian_move(self, x, y, z, speed=0.25):
        self.get_logger().info(f"Cartesian move to ({x}, {y}, {z})")

        pose = self.create_pose(x, y, z)

        req = GetCartesianPath.Request()
        req.group_name = "xarm6"
        req.link_name = "link_tcp"
        req.header.frame_id = "link_base"

        # 🔥 CRITICAL FIX
        req.start_state.is_diff = True

        req.waypoints = [pose.pose]

        req.max_step = 0.02
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()

        if res is None or res.fraction < 0.9:
            self.get_logger().error(f"Cartesian path failed: {res.fraction if res else 'None'}")
            return False

        traj = self.scale_trajectory_speed(res.solution, speed)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj

        future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Execution rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Move complete.")
        return True

    # ==============================
    # API
    # ==============================
    def move_to(self, x, y, z, speed=0.25):
        return self.cartesian_move(x, y, z, speed)

    def home(self):
        return self.move_to(130, 0, 150, speed=0.4)

    def place(self, x, z):
        return self.move_to(x, 220.6, z, speed=0.25)

    # ==============================
    # Gripper
    # ==============================
    def set_gripper(self, pos):
        self.get_logger().info(f"Setting gripper to {pos}")

        goal = GripperCommand.Goal()
        # Scale pos from 0-850, and invert it so 850 maps to 0.0 (open) and 0 maps to 0.850 (closed)
        goal.command.position = (850.0 - float(pos)) / 1000.0
        goal.command.max_effort = -1.0

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Gripper done")
        return True