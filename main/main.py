import argparse
import rclpy
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detection_model.yolo_model import YoloSnapshotNode
from helpers.movement import MIN_Z_MM, MoveArm


class MockMoveArm:
    """Fallback arm controller used when hardware is not connected."""

    def set_gripper(self, pos):
        print(f"[mock] set_gripper({pos})")
        return True

    def home(self):
        print("[mock] home()")
        return True

    def move_to(self, x, y, z, speed=0.7):
        if z < MIN_Z_MM:
            print(f"[mock] REFUSED move_to z={z}, below the {MIN_Z_MM} mm table limit.")
            return False
        print(f"[mock] move_to(x={x}, y={y}, z={z}, speed={speed})")
        return True


# Vertical clearance, in mm, for approaching and retreating from a grasp. Must
# clear the tallest component plus the gripper fingers.
APPROACH_HEIGHT = 80

# One drop-off bin per component type: (x, y, release z) in mm in link_base.
# Release z is above the bin floor, so parts stack rather than being dragged
# through each other. Tune all of these to the bins actually on the table.
PLACE_BINS = {
    "capacitor": (70.0, 220.6, 60.0),
    "resistor": (170.0, 220.6, 60.0),
    "transformer": (270.0, 220.6, 60.0),
}
# Anything the model reports that has no bin above.
REJECT_BIN = (370.0, 220.6, 60.0)


def order_targets(targets):
    """Greedy nearest-neighbour ordering, starting from the base origin.

    Targets are (x, y, z, class); only the coordinates decide the order.
    """
    remaining = list(targets)
    path = []
    curr = (0.0, 0.0, 0.0)
    while remaining:
        closest = min(remaining, key=lambda t: sum((a - b) ** 2 for a, b in zip(t[:3], curr)))
        remaining.remove(closest)
        path.append(closest)
        curr = closest[:3]
    return path


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
        default=None,
        help=(
            'Only pick this class (e.g. "capacitor", "resistor", "transformer"). '
            "Default: pick every detected class, sorting each into its own bin."
        ),
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

    while cli_args.no_hardware or rclpy.ok():
        if cli_args.no_hardware:
            targets = [
                (180.0, -25.0, 30.0, "capacitor"),
                (210.0, 15.0, 32.0, "resistor"),
                (195.0, 40.0, 31.0, "widget"),
                (170.0, 5.0, 6.0, "resistor"),  # low enough to hit the table clamp
            ]
            print(f"[mock] using {len(targets)} synthetic target(s): {targets}")
        else:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.target_positions is None:
                continue
            targets = node.target_positions

        for x, y, z, cls in order_targets(targets):
            grasp_x = x + offset_x
            grasp_z = z + offset_z

            # Reach as deep as the table limit allows rather than giving up on
            # the component. move_to() still refuses anything below the floor.
            if grasp_z < MIN_Z_MM:
                print(
                    f"Target z={z:.1f} mm would grasp at {grasp_z:.1f} mm, below the "
                    f"{MIN_Z_MM} mm table limit. Clamping to {MIN_Z_MM} mm."
                )
                grasp_z = MIN_Z_MM

            print(f"Captured target in main: {x}, {y}, {z} ({cls})")

            # Pick: settle above the component, drop straight down onto it,
            # grip, then lift clear before travelling anywhere sideways.
            moveto.move_to(grasp_x, y, grasp_z + APPROACH_HEIGHT)
            moveto.move_to(grasp_x, y, grasp_z)
            moveto.set_gripper(0)
            moveto.move_to(grasp_x, y, grasp_z + APPROACH_HEIGHT)

            bin_x, bin_y, bin_z = PLACE_BINS.get(cls.lower(), REJECT_BIN)
            if cls.lower() not in PLACE_BINS:
                print(f"No bin for class '{cls}', using the reject bin.")

            # Place: travel high over the bin, lower, release, lift back clear.
            moveto.move_to(bin_x, bin_y, bin_z + APPROACH_HEIGHT)
            moveto.move_to(bin_x, bin_y, bin_z)
            moveto.set_gripper(850)
            moveto.move_to(bin_x, bin_y, bin_z + APPROACH_HEIGHT)
            moveto.home()

        if cli_args.no_hardware:
            print("[mock] demo cycle completed.")
            break

        node.target_positions = None

    if not cli_args.no_hardware:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
