import collections
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import GripperCommand
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath

from moveit_msgs.msg import PlanningScene, CollisionObject, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive



def downward_orientation(yaw=0.0):
    """(x, y, z, w) quaternion for the gripper pointing straight down.

    That is a 180 deg roll about X, then `yaw` radians about the vertical. At
    yaw 0 the fingers close along link_base Y (see helpers/grasp_yaw.py).
    """
    return (math.cos(yaw / 2.0), math.sin(yaw / 2.0), 0.0, 0.0)


# Measured table contact, in mm in link_base: the gripper was jogged down until
# it touched the table and link_base -> link_tcp read z = -0.004 m. Re-measure
# after any change to the table, the mount, or the gripper fingers.
TABLE_CONTACT_Z_MM = -4.0

# Clearance held above that contact point. Raise it if the gripper still grazes
# the table; lower it if grasps on genuinely thin components get refused.
# 10 mm since 2026-10-04: at 6 mm the fingertips all but touched a PCB lying on
# the table, whose solder joints stand a few mm above the table itself.
TABLE_CLEARANCE_MM = 10.0

# Hard floor for the TCP. Every move goes through move_to(), so this is the one
# place the arm is stopped from being driven into the table.
MIN_Z_MM = TABLE_CONTACT_Z_MM + TABLE_CLEARANCE_MM

# Speed of the descent onto a part, as a fraction of the speed MoveIt plans
# at. Until 2026-10-11 every move ran at this (about 18 s per pick); 0.7 for
# every move was too fast on the rig.
DEFAULT_SPEED = 0.3

# Speed of the moves that are clear of the board: getting above a part,
# lifting away from it, and everything to and from the bins and home. Only the
# descent onto a part runs at DEFAULT_SPEED.
TRAVEL_SPEED = 0.6

# A guarded move is stopped when the motor current of the shoulder or elbow
# joint changes by more than this within GUARD_WINDOW_S: the arm has come down
# on something. The 25 free descents of 2026-10-11 changed by 0.39-0.58 A;
# pressing on the gripper by hand gave 2-5 A. picks.csv logs the largest
# change of every descent (descent_current_a).
BLOCKED_CURRENT_A = 1.2
GUARD_WINDOW_S = 0.2
# Nothing is judged while the move is still getting under way.
GUARD_SETTLE_S = 0.3
GUARD_RATE_HZ = 100.0
GUARDED_JOINTS = (1, 2)  # joint2 (shoulder) and joint3 (elbow)

# The measured currents only come over the arm's own SDK, in its 100 Hz "dev"
# report with the report switched from torque to current. What ROS publishes
# as joint effort is a modelled torque that does not react to contact at all
# (checked 2026-10-11 by pressing on the gripper). The connection is only read
# from; all motion still goes through MoveIt.
XARM_IP = "192.168.1.225"


class TorqueGuard:
    """Watches joint currents during one move for the jump of a collision."""

    def __init__(self):
        self.samples = collections.deque()  # (seconds since start, currents)
        self.peak = 0.0

    def update(self, t, currents):
        """Feed one reading taken `t` seconds into the move. True once a
        current has changed by more than BLOCKED_CURRENT_A within the window."""
        if t < GUARD_SETTLE_S:
            return False
        now = [currents[j] for j in GUARDED_JOINTS]
        self.samples.append((t, now))
        while t - self.samples[0][0] > GUARD_WINDOW_S:
            self.samples.popleft()
        before = self.samples[0][1]
        self.peak = max(self.peak, max(abs(a - b) for a, b in zip(now, before)))
        return self.peak > BLOCKED_CURRENT_A


# Rest pose returned to between picks, (x, y, z) in mm in link_base. Held high
# so the arm sits clear of the table and out of the camera's view of it.
HOME_POSE_MM = (130.0, 0.0, 300.0)


class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm_node")

        # Finger opening in mm reported after the last gripper command. After a
        # grip this is the width of what is held; near 0 means it closed on
        # nothing.
        self.gripper_width_mm = None

        # Set by a guarded move_to(): whether it was stopped because the arm
        # came down on something, and the largest current change it saw (A).
        self.blocked = False
        self.descent_current_a = None
        self._guard = None
        self._guard_started = None
        self._goal_handle = None
        self._sdk = self._connect_sdk()
        self.create_timer(1.0 / GUARD_RATE_HZ, self._poll_guard)

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
    # Pose (pointing down, rotated by yaw)
    # ==============================
    def create_pose(self, x, y, z, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "link_base"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x / 1000.0
        pose.pose.position.y = y / 1000.0
        pose.pose.position.z = z / 1000.0

        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = downward_orientation(yaw)

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
    def _connect_sdk(self):
        """Read-only connection for the measured joint currents, or None."""
        try:
            from xarm.wrapper import XArmAPI
            sdk = XArmAPI(XARM_IP, report_type="dev")
            if not sdk.connected or sdk.set_report_tau_or_i(1) != 0:
                raise RuntimeError("no connection, or the current report was refused")
            return sdk
        except Exception as e:
            self.get_logger().warn(
                f"No joint currents from the arm ({e}); descents will not be guarded.")
            return None

    def _poll_guard(self):
        if self._guard is None or self._sdk is None:
            return
        t = time.monotonic() - self._guard_started
        if self._guard.update(t, self._sdk.joints_torque) and not self.blocked:
            self.blocked = True
            self.get_logger().warn(
                f"Current jumped {self._guard.peak:.2f} A - the arm is on something. Stopping.")
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()

    def move_to(self, x, y, z, speed=DEFAULT_SPEED, yaw=0.0, guard=False):
        """Straight-line move of the TCP, gripper pointing down. With `guard`
        the move is stopped as soon as the arm pushes against something;
        `self.blocked` then says so. Returns whether the target was reached."""
        self.get_logger().info(
            f"Cartesian move to ({x}, {y}, {z}), yaw {math.degrees(yaw):.0f} deg")
        self.blocked = False

        # Refused, not clamped: a silently raised grasp closes on nothing, which
        # is a confusing failure. Below the floor means the target is wrong.
        if z < MIN_Z_MM:
            self.get_logger().error(
                f"Refusing move to z={z} mm, below the {MIN_Z_MM} mm table limit."
            )
            return False

        pose = self.create_pose(x, y, z, yaw)

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

        if guard:
            self._guard, self._guard_started = TorqueGuard(), time.monotonic()
        try:
            result = self._send_goal(self.execute_client, goal, "Trajectory execution")
        finally:
            if guard:
                self.descent_current_a = self._guard.peak
                self._guard = None
        if result is None:
            return False
        if self.blocked:
            return False

        # An accepted goal can still fail to run - e.g. the trajectory
        # controller is inactive after the arm was put in manual mode.
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Trajectory execution failed (MoveIt error {code}).")
            return False

        self.get_logger().info("Move complete.")
        return True

    def _send_goal(self, client, goal, what):
        """Send an action goal and block until it finishes.

        Returns the action result, or None if the goal never ran.
        """
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{what} rejected.")
            return None

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._goal_handle = None
        return result_future.result().result

    def home(self):
        return self.move_to(*HOME_POSE_MM, speed=TRAVEL_SPEED)

    # ==============================
    # Gripper
    # ==============================
    def set_gripper(self, pos):
        self.get_logger().info(f"Setting gripper to {pos}")

        goal = GripperCommand.Goal()
        # Scale pos from 0-850, and invert it so 850 maps to 0.0 (open) and 0 maps to 0.850 (closed)
        goal.command.position = (850.0 - float(pos)) / 1000.0
        goal.command.max_effort = -1.0

        result = self._send_goal(self.gripper_client, goal, "Gripper command")
        if result is None:
            return False
        self.gripper_width_mm = (850.0 - result.position * 1000.0) / 10.0

        self.get_logger().info("Gripper done")
        return True
