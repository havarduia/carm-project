import argparse
import rclpy
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detection_model.yolo_model import YoloSnapshotNode
from helpers.movement import MoveArm


class MockMoveArm:
    """Fallback arm controller used when hardware is not connected."""

    def set_gripper(self, pos):
        print(f"[mock] set_gripper({pos})")
        return True

    def home(self):
        print("[mock] home()")
        return True

    def move_to(self, x, y, z, speed=0.7):
        print(f"[mock] move_to(x={x}, y={y}, z={z}, speed={speed})")
        return True

    def place(self, x, z):
        print(f"[mock] place(x={x}, z={z})")
        return True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="xArm pick-and-place workflow")
    parser.add_argument(
        "--no-hardware",
        action="store_true",
        help=(
            "Run without requiring RealSense or xArm hardware. "
            "Uses mocked detections and mocked arm commands."
        ),
    )
    parser.add_argument(
        "--target-class",
        default="capacitor",
        help='Target class to detect (e.g. "capacitor", "resistor", "transformer").',
    )
    return parser.parse_args(argv)

def main(args=None):
    cli_args = parse_args(args)
    offset_x = 10
    offset_z = -10

    if cli_args.no_hardware:
        print("Running in --no-hardware mode (mock camera + mock xArm).")
        node = None
        moveto = MockMoveArm()
    else:
        rclpy.init(args=args)
        node = YoloSnapshotNode(target_class=cli_args.target_class)
        # Initialize MoveIt arm wrapper
        print("Initializing MoveIt planner for arm...")
        moveto = MoveArm()

    moveto.set_gripper(850)
    moveto.home()

    current_place_x = 70  # Starting x coordinate for placement
    step_size = 50       # Amount to increase x each cycle

    while cli_args.no_hardware or rclpy.ok():
        if cli_args.no_hardware:
            targets = [(180.0, -25.0, 30.0), (210.0, 15.0, 32.0)]
            print(f"[mock] using {len(targets)} synthetic target(s): {targets}")
        else:
            rclpy.spin_once(node, timeout_sec=0.1)
            if getattr(node, "target_positions", None) is None:
                continue
            targets = node.target_positions.copy()

        path = []
        curr = (0, 0, 0)
        while targets:
            closest = min(targets, key=lambda p: (p[0]-curr[0])**2 + (p[1]-curr[1])**2 + (p[2]-curr[2])**2)
            targets.remove(closest)
            path.append(closest)
            curr = closest

        for x, y, z in path:
            if z < 5:
                print("Invalid target position detected, skipping...")
                continue
            print(f"Captured target in main: {x}, {y}, {z}")

            moveto.move_to(x + offset_x, y, z + offset_z)
            moveto.set_gripper(0)
            moveto.move_to(x + offset_x, y, z + offset_z + 80)

            current_place_x += step_size
            moveto.place(current_place_x, z=z + offset_z + 80)
            moveto.place(current_place_x, z=z + offset_z)
            moveto.set_gripper(850)
            moveto.home()

        if cli_args.no_hardware:
            print("[mock] demo cycle completed.")
            break

        node.target_positions = None

    if not cli_args.no_hardware:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
