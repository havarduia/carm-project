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

# The gripper always approaches straight down: 180 deg roll about X.
# (x, y, z, w) quaternion for that fixed orientation.
DOWNWARD_ORIENTATION = (1.0, 0.0, 0.0, 0.0)

# Measured table contact, in mm in link_base: the gripper was jogged down until
# it touched the table and link_base -> link_tcp read z = -0.004 m. Re-measure
# after any change to the table, the mount, or the gripper fingers.
TABLE_CONTACT_Z_MM = -4.0

# Clearance held above that contact point. Raise it if the gripper still grazes
# the table; lower it if grasps on genuinely thin components get refused.
TABLE_CLEARANCE_MM = 6.0

# Hard floor for the TCP. Every move goes through move_to(), so this is the one
# place the arm is stopped from being driven into the table.
MIN_Z_MM = TABLE_CONTACT_Z_MM + TABLE_CLEARANCE_MM

# Rest pose returned to between picks, (x, y, z) in mm in link_base. Held high
# so the arm sits clear of the table and out of the camera's view of it.
HOME_POSE_MM = (130.0, 0.0, 300.0)


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

        self._add_collision_object(
            "camera_pole", SolidPrimitive.CYLINDER, [0.78, 0.02], (0.053, -0.175, 0.78 / 2.0)
        )
        time.sleep(1)
        self._add_collision_object(
            "camera_ball", SolidPrimitive.SPHERE, [0.06], (0.111, -0.1504, 0.35)
        )
        time.sleep(1)

    def _add_collision_object(self, object_id, shape_type, dimensions, position):
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = "link_base"

        primitive = SolidPrimitive()
        primitive.type = shape_type
        primitive.dimensions = dimensions

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0

        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        self.scene_pub.publish(scene)
        self.get_logger().info(f"{object_id} added.")

    # ==============================
    # Pose (fixed downward)
    # ==============================
    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "link_base"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x / 1000.0
        pose.pose.position.y = y / 1000.0
        pose.pose.position.z = z / 1000.0

        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = DOWNWARD_ORIENTATION

        return pose

    # ==============================
    # Speed scaling
    # ==============================
    def scale_trajectory_speed(self, traj, scale):
        if scale <= 0.0:
            raise ValueError(f"speed scale must be > 0, got {scale}")
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
    # Cartesian move
    # ==============================
    def move_to(self, x, y, z, speed=0.7):
        self.get_logger().info(f"Cartesian move to ({x}, {y}, {z})")

        # Refused, not clamped: a silently raised grasp closes on nothing, which
        # is a confusing failure. Below the floor means the target is wrong.
        if z < MIN_Z_MM:
            self.get_logger().error(
                f"Refusing move to z={z} mm, below the {MIN_Z_MM} mm table limit."
            )
            return False

        pose = self.create_pose(x, y, z)

        req = GetCartesianPath.Request()
        req.group_name = "xarm6"
        req.link_name = "link_tcp"
        req.header.frame_id = "link_base"
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

        if not self._send_goal(self.execute_client, goal, "Trajectory execution"):
            return False

        self.get_logger().info("Move complete.")
        return True

    def _send_goal(self, client, goal, what):
        """Send an action goal and block until it finishes. False if it never ran."""
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{what} rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def home(self):
        return self.move_to(*HOME_POSE_MM, speed=0.7)

    # ==============================
    # Gripper
    # ==============================
    def set_gripper(self, pos):
        self.get_logger().info(f"Setting gripper to {pos}")

        goal = GripperCommand.Goal()
        # Scale pos from 0-850, and invert it so 850 maps to 0.0 (open) and 0 maps to 0.850 (closed)
        goal.command.position = (850.0 - float(pos)) / 1000.0
        goal.command.max_effort = -1.0

        if not self._send_goal(self.gripper_client, goal, "Gripper command"):
            return False

        self.get_logger().info("Gripper done")
        return True
