import argparse
import math
import rclpy
import sys
import os
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detection_model.yolo_model import YoloSnapshotNode, read_line
from helpers.grasp_yaw import choose_grasp_yaw
from helpers.movement import DEFAULT_SPEED, MIN_Z_MM, TABLE_CONTACT_Z_MM, MoveArm
from helpers.run_log import RunLog


class MockMoveArm:
    """Fallback arm controller used when hardware is not connected."""

    gripper_width_mm = None

    def set_gripper(self, pos):
        print(f"[mock] set_gripper({pos})")
        return True

    def home(self):
        print("[mock] home()")
        return True

    def move_to(self, x, y, z, speed=DEFAULT_SPEED, yaw=0.0):
        if z < MIN_Z_MM:
            print(f"[mock] REFUSED move_to z={z}, below the {MIN_Z_MM} mm table limit.")
            return False
        print(f"[mock] move_to(x={x}, y={y}, z={z}, speed={speed}, yaw={yaw})")
        return True


# Vertical clearance, in mm, for approaching and retreating from a grasp. Must
# clear the tallest component plus the gripper fingers.
APPROACH_HEIGHT = 80

# Gripper opening used for picking and releasing, in the gripper's 0-850 units
# (0.1 mm between the fingers). Half open: the fully open fingers sweep twice
# the width and knock neighbouring components.
GRIPPER_OPEN = 400

# One drop-off bin per component type: (x, y, release z) in mm in link_base.
# Release z is above the bin floor, so parts stack rather than being dragged
# through each other. Tune all of these to the bins actually on the table.
PLACE_BINS = {
    # Checked on the rig 2026-10-04: centred on the bin, releasing 3 mm above
    # the rim of a 50 mm deep bin whose rim is at z = 57 mm.
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


# What the code cannot see for itself, asked after each snapshot's picks:
# (column in snapshots.csv, question).
OPERATOR_QUESTIONS = [
    ("missed_by_camera", "Components on the table that the camera did NOT detect"),
    ("false_detections", "Detections that were not a real component"),
    ("grasp_failures", "Picks where the gripper came up empty or dropped the part"),
    ("wrong_bin", "Parts released outside the correct bin"),
    ("knocked_over", "Neighbouring components knocked or moved"),
]


def ask_count(question):
    """The operator's count, 0 on a bare Enter, or None with no terminal to ask on."""
    while True:
        answer = read_line(f"  {question}? [0] ")
        if answer is None:
            return None
        if not answer.strip():
            return 0
        if answer.strip().isdigit():
            return int(answer)
        print("  Please type a whole number.")


def pick_and_place(moveto, grasp, yaw, bin_pose):
    """One pick-and-place. Returns (name of the move that failed or None,
    finger opening in mm after gripping)."""
    gx, gy, gz = grasp
    bx, by, bz = bin_pose

    # Pick: settle above the component, drop straight down onto it,
    # grip, then lift clear before travelling anywhere sideways.
    if not moveto.move_to(gx, gy, gz + APPROACH_HEIGHT, yaw=yaw):
        return "above part", None
    if not moveto.move_to(gx, gy, gz, yaw=yaw):
        return "down to part", None
    moveto.set_gripper(0)
    grip_width = moveto.gripper_width_mm
    if not moveto.move_to(gx, gy, gz + APPROACH_HEIGHT, yaw=yaw):
        return "lift", grip_width

    # Place: travel high over the bin, lower, release, lift back clear.
    if not moveto.move_to(bx, by, bz + APPROACH_HEIGHT):
        return "above bin", grip_width
    if not moveto.move_to(bx, by, bz):
        return "down to bin", grip_width
    moveto.set_gripper(GRIPPER_OPEN)
    if not moveto.move_to(bx, by, bz + APPROACH_HEIGHT):
        return "lift from bin", grip_width
    return None, grip_width


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
    # Grasp offsets in mm, from the hover check of 2026-10-04: the detected
    # point is on the side of the component facing the camera, which left the
    # gripper 5-7.5 mm short in y and already centred in x.
    offset_x = 0
    offset_y = 6
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

    moveto.set_gripper(GRIPPER_OPEN)
    moveto.home()

    log = RunLog()
    print(f"Logging this run to {log.dir}")
    snapshot_no = 0

    while cli_args.no_hardware or rclpy.ok():
        if cli_args.no_hardware:
            targets = [
                (180.0, -25.0, 30.0, "capacitor"),
                (210.0, 15.0, 32.0, "resistor"),
                (195.0, 40.0, 31.0, "widget"),
                (170.0, 5.0, 6.0, "resistor"),  # low enough to hit the table clamp
            ]
            detected = len(targets)
            print(f"[mock] using {len(targets)} synthetic target(s): {targets}")
        else:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.snapshot is None:
                continue
            snapshot, node.snapshot = node.snapshot, None
            targets, detected = snapshot['targets'], snapshot['detected']

        snapshot_no += 1
        if not cli_args.no_hardware:
            log.save_image(f"snapshot_{snapshot_no:02d}_raw.jpg", snapshot['raw'])
            log.save_image(f"snapshot_{snapshot_no:02d}_detections.jpg", snapshot['annotated'])

        outcomes = []
        # A failed move can leave the gripper closed on a part.
        moveto.set_gripper(GRIPPER_OPEN)

        for attempt, (x, y, z, cls) in enumerate(order_targets(targets), 1):
            grasp_x = x + offset_x
            grasp_y = y + offset_y
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

            started = time.time()
            failed_step, grip_width = None, None

            # Turn the gripper so its open fingers come down on free space.
            yaw = 0.0
            if node is not None and node.cloud_mm is not None:
                yaw = choose_grasp_yaw(node.cloud_mm, grasp_x, grasp_y, grasp_z,
                                       GRIPPER_OPEN / 10.0, TABLE_CONTACT_Z_MM)

            if yaw is None:
                outcome = "no_free_yaw"
                print("No gripper angle clears the neighbours, skipping it. "
                      "Take a new snapshot once they are picked.")
            else:
                print(f"Gripper yaw: {math.degrees(yaw):.0f} deg")

                bin_pose = PLACE_BINS.get(cls.lower(), REJECT_BIN)
                if cls.lower() not in PLACE_BINS:
                    print(f"No bin for class '{cls}', using the reject bin.")

                failed_step, grip_width = pick_and_place(
                    moveto, (grasp_x, grasp_y, grasp_z), yaw, bin_pose)
                outcome = "placed" if failed_step is None else "move_failed"
                moveto.home()

            outcomes.append(outcome)
            log.write("picks", {
                "time": time.strftime("%H:%M:%S"),
                "snapshot": snapshot_no,
                "attempt": attempt,
                "class": cls,
                "x": round(x, 1), "y": round(y, 1), "z": round(z, 1),
                "grasp_x": round(grasp_x, 1), "grasp_y": round(grasp_y, 1),
                "grasp_z": round(grasp_z, 1),
                "yaw_deg": "" if yaw is None else round(math.degrees(yaw)),
                "outcome": outcome,
                "failed_step": failed_step or "",
                "grip_width_mm": "" if grip_width is None else round(grip_width, 1),
                "seconds": round(time.time() - started, 1),
            })

            if failed_step is not None:
                # The arm is not following commands; trying the rest would
                # only close the gripper on thin air.
                print(f"Move '{failed_step}' failed - stopping this snapshot's picks. "
                      "Check the arm (and the gripper, which may still hold a part).")
                break

        row = {
            "time": time.strftime("%H:%M:%S"),
            "snapshot": snapshot_no,
            "target_class": cli_args.target_class or "all",
            "detected": detected,
            "located": len(targets),
            "attempted": len(outcomes),
            "placed": outcomes.count("placed"),
            "no_free_yaw": outcomes.count("no_free_yaw"),
            "move_failed": outcomes.count("move_failed"),
        }
        print(f"Snapshot {snapshot_no}: {row['detected']} detected, {row['located']} located, "
              f"{row['placed']} placed, {row['no_free_yaw']} skipped (no free angle), "
              f"{row['move_failed']} failed move.")

        answers = {}
        if not cli_args.no_hardware:
            print("How did it go? Type a number, or Enter for 0.")
            answers = {key: ask_count(question) for key, question in OPERATOR_QUESTIONS}
        for key, _ in OPERATOR_QUESTIONS:
            answer = answers.get(key)
            row[key] = "" if answer is None else answer
        if answers.get("grasp_failures") is not None and answers.get("wrong_bin") is not None:
            row["successes"] = row["placed"] - answers["grasp_failures"] - answers["wrong_bin"]
        else:
            row["successes"] = ""
        row["note"] = (read_line("  Note (optional): ") or "") if answers else ""
        log.write("snapshots", row)

        if cli_args.no_hardware:
            print("[mock] demo cycle completed.")
            break

        print("Press 's' for the next snapshot, 'q' to quit.")
        node.target_positions = None

    log.close()
    print(f"Run log saved in {log.dir}")

    if not cli_args.no_hardware:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
