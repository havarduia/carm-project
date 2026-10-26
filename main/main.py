import argparse
import math
import rclpy
import sys
import os
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detection_model.yolo_model import YoloSnapshotNode, read_line
from helpers.grasp_yaw import choose_grasp_yaw
from helpers.part_top import part_top
from helpers.segment import segment_top
from helpers.movement import (DEFAULT_SPEED, HOME_POSE_MM, MIN_Z_MM, TABLE_CONTACT_Z_MM,
                              TRAVEL_SPEED, MoveArm)
from helpers.run_log import RunLog


class MockMoveArm:
    """Fallback arm controller used when hardware is not connected."""

    gripper_width_mm = None
    blocked = False
    descent_current_a = None

    def set_gripper(self, pos):
        print(f"[mock] set_gripper({pos})")
        return True

    def home(self):
        print("[mock] home()")
        return True

    def move_to(self, x, y, z, speed=DEFAULT_SPEED, yaw=0.0, guard=False):
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

# Opening used for the descent onto a part: its width from the detection box
# plus this much free space on each side, and never less than
# MIN_GRIPPER_OPEN. The slack covers the arm arriving a few mm off the part's
# centre: 2-3 mm in the hover checks of 2026-10-11, once the trim in main()
# was set. If no gripper angle is free at that opening, wider ones are
# tried in steps of OPENING_STEP up to GRIPPER_OPEN, or the part's own
# opening where that is wider.
FINGER_CLEARANCE_MM = 6.0
MIN_GRIPPER_OPEN = 200
# Widest opening for a descent (60 mm). Wider than GRIPPER_OPEN: at 40 mm,
# parts over ~32 mm wide (transformers, big capacitors) blocked every yaw
# themselves and were never tried (no_free_yaw, 2026-10-14). Only a part that
# wide opens this far: a 14 mm capacitor widened to 60 mm to clear its
# neighbours, closed through them and pushed them over (2026-10-26).
MAX_GRASP_OPEN = 600
OPENING_STEP = 50

# The fingertips stop this far below a part's top face, or at the table limit
# if that comes first.
# 10 mm since 2026-10-26: at 15 the fingers slid far down the sides of tall
# capacitors, into the parts packed around them. Shallower also lets them pass
# over more of the short neighbours (see NEIGHBOUR_BELOW_MM in grasp_yaw.py).
GRASP_DEPTH_MM = 10.0

# With --hover the fingertips stop this far above the part's top face.
HOVER_HEIGHT_MM = 10.0

# The detection box reads wider than the part, because the camera looks at it
# from the side: the gripper closed to 0.6-0.8 of the box width on the 25
# picks of 2026-10-11. Used for how much room a neighbour takes up; the
# opening for the part itself keeps the full box width, as extra clearance.
BOX_TO_PART_WIDTH = 0.75

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

# Where the board lies: (x min, x max), (y min, y max) in mm in link_base.
# Detections outside it are ignored - above all the parts already lying in the
# bins, which the camera also sees. Picks of 2026-10-11 spanned x 305-465,
# y -67 to 87; the bins start at about y 170.
PICK_AREA_MM = ((150.0, 600.0), (-250.0, 150.0))

# The gripper reports its finger opening after closing. Below this it closed
# on nothing: the empty closes of 2026-10-11 read 0.0-0.1 mm, held parts
# 7 mm and up (one read 1.5 mm, which may have been a part held by a lead).
EMPTY_GRIP_MM = 1.0

# After every pick the board is detected again, so a part that is still there
# is tried again. Give up on a spot after this many tries, counting detections
# within SAME_PART_RADIUS_MM as the same part.
MAX_ATTEMPTS_PER_PART = 2
SAME_PART_RADIUS_MM = 15.0


# Parts whose tops are within this of each other count as equally tall when
# choosing what to pick next.
HEIGHT_BAND_MM = 10.0


def pick_order(located_parts):
    """Tallest first; among parts of about the same height, the one nearest
    the home pose, where every pick starts from.

    A tall part is in the way of the fingers reaching for its neighbours and
    hides what stands behind it from the camera, so it goes first. Each item
    is (target, top): the (x, y, z, class, ...) target and the centre of its
    top face, or None where that was not found.
    """
    def key(part):
        target, top = part
        x, y, z = target[:3] if top is None else top
        return -round(z / HEIGHT_BAND_MM), math.dist((x, y), HOME_POSE_MM[:2])
    return sorted(located_parts, key=key)


def in_pick_area(targets):
    """The targets lying inside PICK_AREA_MM."""
    (x_min, x_max), (y_min, y_max) = PICK_AREA_MM
    return [t for t in targets if x_min <= t[0] <= x_max and y_min <= t[1] <= y_max]


def attempts_near(target, attempted, targets=()):
    """How many of the (x, y) spots in `attempted` are this target's spot.

    A spot counts only for the part of `targets` nearest to it: two parts
    standing against each other are closer than SAME_PART_RADIUS_MM, and a
    part tried twice must not use up its neighbour's tries (2026-10-26)."""
    def mine(spot):
        return all(math.dist(target[:2], spot) <= math.dist(t[:2], spot) for t in targets)
    return sum(math.dist(target[:2], spot) < SAME_PART_RADIUS_MM and mine(spot)
               for spot in attempted)


def finger_opening(width_mm):
    """Gripper opening (0-850 units) to descend with onto a part `width_mm`
    wide. Fully open to GRIPPER_OPEN when the width is unknown."""
    if width_mm is None:
        return GRIPPER_OPEN
    opening = round((width_mm + 2 * FINGER_CLEARANCE_MM) * 10)
    return min(MAX_GRASP_OPEN, max(MIN_GRIPPER_OPEN, opening))


def choose_grasp(cloud_mm, x, y, z, width_mm, neighbours=()):
    """(opening, yaw) for a grasp at (x, y, z): the narrowest opening, from
    the part's own width upwards, at which some yaw brings the fingers down
    on free space and catches none of `neighbours`, the other detected parts
    as (x, y, radius, top z). None if nothing fits. Without a cloud nothing is
    checked."""
    first = finger_opening(width_mm)
    if cloud_mm is None:
        return first, 0.0
    widest = max(GRIPPER_OPEN, first)
    for opening in list(range(first, widest, OPENING_STEP)) + [widest]:
        yaw = choose_grasp_yaw(cloud_mm, x, y, z, opening / 10.0, TABLE_CONTACT_Z_MM,
                               neighbours)
        if yaw is not None:
            return opening, yaw
    return None


# What the code cannot see for itself, asked after each snapshot's picks:
# (column in snapshots.csv, question).
OPERATOR_QUESTIONS = [
    ("missed_by_camera", "Components on the table that the camera did NOT detect"),
    ("false_detections", "Detections that were not a real component"),
    ("grasp_failures", "Picks logged as placed where the part did not reach a bin"),
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


def pick_and_place(moveto, grasp, yaw, bin_pose, opening=GRIPPER_OPEN):
    """One pick-and-place. Returns (outcome, name of the move that failed or
    None, finger opening in mm after gripping). The outcome is "placed",
    "empty_grasp", "blocked_descent" or "move_failed". `opening` is how far
    the fingers are open on the way down.

    Only the descent onto the part is slow; everything clear of the board
    runs at TRAVEL_SPEED."""
    gx, gy, gz = grasp
    bx, by, bz = bin_pose

    # Pick: settle above the component, drop straight down onto it,
    # grip, then lift clear before travelling anywhere sideways.
    if not moveto.move_to(gx, gy, gz + APPROACH_HEIGHT, speed=TRAVEL_SPEED, yaw=yaw):
        return "move_failed", "above part", None
    moveto.set_gripper(opening)
    if not moveto.move_to(gx, gy, gz, yaw=yaw, guard=True):
        if not moveto.blocked:
            return "move_failed", "down to part", None
        # A finger came down on something. Back off and leave the part.
        print("Descent blocked - lifting off again.")
        if not moveto.move_to(gx, gy, gz + APPROACH_HEIGHT, speed=TRAVEL_SPEED, yaw=yaw):
            return "move_failed", "lift after blocked descent", None
        return "blocked_descent", None, None
    moveto.set_gripper(0)
    grip_width = moveto.gripper_width_mm
    empty = grip_width is not None and grip_width < EMPTY_GRIP_MM
    if empty:
        # Nothing to carry to the bin. Open again so the fingers do not drag
        # anything up with them.
        print(f"Gripper closed to {grip_width:.1f} mm - nothing in it.")
        moveto.set_gripper(opening)
    if not moveto.move_to(gx, gy, gz + APPROACH_HEIGHT, speed=TRAVEL_SPEED, yaw=yaw):
        return "move_failed", "lift", grip_width
    if empty:
        return "empty_grasp", None, grip_width

    # Place: travel high over the bin, lower, release, lift back clear.
    if not moveto.move_to(bx, by, bz + APPROACH_HEIGHT, speed=TRAVEL_SPEED):
        return "move_failed", "above bin", grip_width
    if not moveto.move_to(bx, by, bz, speed=TRAVEL_SPEED):
        return "move_failed", "down to bin", grip_width
    moveto.set_gripper(max(GRIPPER_OPEN, opening))
    if not moveto.move_to(bx, by, bz + APPROACH_HEIGHT, speed=TRAVEL_SPEED):
        return "move_failed", "lift from bin", grip_width
    return "placed", None, grip_width


def hover(moveto, x, y, top_z, yaw, opening):
    """Hold the open gripper HOVER_HEIGHT_MM above a part until Enter is
    pressed, to see by eye how well the arm lines up with it. Nothing is
    gripped. Returns the name of the move that failed, or None."""
    if not moveto.move_to(x, y, top_z + APPROACH_HEIGHT, speed=TRAVEL_SPEED, yaw=yaw):
        return "above part"
    moveto.set_gripper(opening)
    if not moveto.move_to(x, y, top_z + HOVER_HEIGHT_MM, yaw=yaw, guard=True):
        return "down to hover"
    read_line("  Hovering. Look where the fingers are, then press Enter to lift. ")
    if not moveto.move_to(x, y, top_z + APPROACH_HEIGHT, speed=TRAVEL_SPEED, yaw=yaw):
        return "lift"
    return None


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
    parser.add_argument(
        "--hover",
        action="store_true",
        help=(
            "Calibration check: per snapshot, hold the open gripper just above the "
            "first part instead of picking, so the alignment can be judged by eye."
        ),
    )
    return parser.parse_args(argv)

def main(args=None):
    cli_args = parse_args(args)
    # Where the arm goes relative to where the camera says the part's top
    # face is, in mm: the error left by the hand-eye calibration. From the
    # hover check of 2026-10-11 at (385, 1): with the y trim at 6 the part
    # stood about 14 mm towards the base and 4 mm towards the bins of the
    # gripper's centre (phone photos of a 13 mm part, +/- 3 mm). One spot only.
    # offset_z is only used when the top face cannot be found in the depth
    # points (see helpers/segment.py and helpers/part_top.py).
    offset_x = -14
    offset_y = 10
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

    def locate(target):
        """(target, centre of its top face or None): from the part cut out of
        the depth image (helpers/segment.py), else from the points around the
        detection (helpers/part_top.py)."""
        x, y, z, _, *size = target
        if node is None or node.cloud_mm is None or not size:
            return target, None
        top = segment_top(node.grid_mm, size[1]) if len(size) > 1 else None
        return target, top or part_top(node.cloud_mm, (x, y, z), size[0] / 2.0,
                                       node.camera_xy_mm)

    def neighbours_of(xy, located_parts):
        """Every other detected part as (x, y, radius, top z), where the camera
        sees it. The top is None where it was not found."""
        others = []
        for (x, y, _, _, *size), top in located_parts:
            if (x, y) == xy or not size:
                continue
            nx, ny, top_z = (x, y, None) if top is None else top
            others.append((nx, ny, BOX_TO_PART_WIDTH * size[0] / 2.0, top_z))
        return others

    log = RunLog()
    print(f"Logging this run to {log.dir}")
    # One 's' press is one trial. Counted and totalled here so the operator
    # does not have to; it starts again from 1 each time main.py is started.
    snapshot_no = 0
    trial = "Hover check" if cli_args.hover else "Trial"
    total_placed = total_located = 0

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
            last_annotated = snapshot['annotated']
        snapshot_targets = targets

        snapshot_no += 1
        print(f"\n===== {trial} {snapshot_no} =====")
        if not cli_args.no_hardware:
            log.save_image(f"snapshot_{snapshot_no:02d}_raw.jpg", snapshot['raw'])
            log.save_image(f"snapshot_{snapshot_no:02d}_detections.jpg", snapshot['annotated'])

        targets = in_pick_area(targets)
        if len(targets) < len(snapshot_targets):
            print(f"Ignoring {len(snapshot_targets) - len(targets)} detection(s) outside the pick area.")
        located = len(targets)

        outcomes = []
        # (x, y) of every pick on this board that did not place its part. A
        # placed part is gone; a part found at its spot later is another one,
        # revealed by it, and starts with no tries.
        # ponytail: a pick logged as placed that left its part behind is retried
        # without limit; count placed picks too if that is ever seen.
        attempted = []
        attempt = 0
        # A failed move can leave the gripper closed on a part.
        moveto.set_gripper(GRIPPER_OPEN)

        # One pick per detection: the board is detected again after each one,
        # so a part pushed by the last pick is found where it now lies and a
        # part that is still there is tried again.
        while targets:
            chosen, blocked, crowded = None, [], False
            located_parts = [locate(t) for t in targets]
            for (x, y, z, cls, *size), top in pick_order(located_parts):
                width_mm = size[0] if size else None
                if attempts_near((x, y), attempted, targets) >= MAX_ATTEMPTS_PER_PART:
                    continue

                # Grasp at the centre of the part's top face, a fixed depth
                # below it. Without depth to find it, fall back to the
                # detector's point and the fixed offsets.
                if top is not None:
                    seen_x, seen_y, top_z = top
                    grasp_z = top_z - GRASP_DEPTH_MM
                else:
                    seen_x, seen_y, top_z = x, y, None
                    grasp_z = z + offset_z
                # Where the camera sees the part, and where the arm has to go
                # to be there. Everything checked against the depth points
                # uses the first; only the arm is sent to the second.
                grasp_x = seen_x + offset_x
                grasp_y = seen_y + offset_y

                # Reach as deep as the table limit allows rather than giving up on
                # the component. move_to() still refuses anything below the floor.
                if grasp_z < MIN_Z_MM:
                    print(
                        f"Target at z={z:.1f} mm would grasp at {grasp_z:.1f} mm, below the "
                        f"{MIN_Z_MM} mm table limit. Clamping to {MIN_Z_MM} mm."
                    )
                    grasp_z = MIN_Z_MM

                # Open the fingers no wider than the part needs, and turn them
                # so they come down on free space.
                grasp = choose_grasp(None if node is None else node.cloud_mm,
                                     seen_x, seen_y, grasp_z, width_mm,
                                     neighbours_of((x, y), located_parts))

                part = (x, y, z, cls, grasp_x, grasp_y, grasp_z, width_mm, top_z)
                if grasp is None:
                    blocked.append((part, (seen_x, seen_y)))
                    continue
                chosen, (opening, yaw) = part, grasp
                break

            # Deadlock: every part left has a neighbour in the way, typically
            # two parts standing against each other, one half hidden behind
            # the other. Leaving them helps nobody, so take the first one
            # (the tallest) regardless of its neighbours. The fingers must
            # still come down on free space as far as the depth points show.
            # One that already came down on its neighbour goes last: taking
            # the neighbour first frees both (2026-10-26, picks 8-10).
            if chosen is None:
                blocked.sort(key=lambda b: attempts_near(b[0][:2], attempted, targets))
                for part, (seen_x, seen_y) in blocked:
                    grasp = choose_grasp(None if node is None else node.cloud_mm,
                                         seen_x, seen_y, part[6], part[7])
                    if grasp is not None:
                        print("Every part left has a neighbour in the way - "
                              "taking the tallest one anyway.")
                        chosen, (opening, yaw) = part, grasp
                        crowded = True
                        break
            # Packed so tight that even then no fingers come down on free
            # space (2026-10-26: six capacitors, nothing picked). Take the
            # first one still, at its own opening, turned where the least is
            # in the way. A blocked descent counts as a try, so this ends.
            if chosen is None and blocked and node is not None:
                part, (seen_x, seen_y) = blocked[0]
                opening = finger_opening(part[7])
                yaw = choose_grasp_yaw(node.cloud_mm, seen_x, seen_y, part[6], opening / 10.0,
                                       TABLE_CONTACT_Z_MM, best_effort=True)
                print("No gripper angle is free anywhere - taking the tallest part "
                      "where the least is in the way.")
                chosen, crowded = part, True
            blocked = [part for part, _ in blocked]

            started = time.time()
            failed_step, grip_width, descent_current = None, None, None

            if chosen is None:
                # Whatever is left is either given up on or boxed in.
                todo, yaw, opening = blocked, None, None
                if blocked:
                    print(f"No gripper angle clears the neighbours of {len(blocked)} part(s), "
                          "leaving them.")
            else:
                todo = [chosen]

            for x, y, z, cls, grasp_x, grasp_y, grasp_z, width_mm, top_z in todo:
                attempt += 1
                if chosen is None:
                    outcome = "no_free_yaw"
                else:
                    print(f"Captured target in main: {x}, {y}, {z} ({cls})")
                    print(f"Gripper yaw: {math.degrees(yaw):.0f} deg, opening {opening / 10:.0f} mm")

                    bin_pose = PLACE_BINS.get(cls.lower(), REJECT_BIN)
                    if cls.lower() not in PLACE_BINS:
                        print(f"No bin for class '{cls}', using the reject bin.")

                    moveto.descent_current_a = None
                    if cli_args.hover:
                        failed_step = hover(moveto, grasp_x, grasp_y,
                                            z if top_z is None else top_z, yaw, opening)
                        outcome = "hover" if failed_step is None else "move_failed"
                    else:
                        outcome, failed_step, grip_width = pick_and_place(
                            moveto, (grasp_x, grasp_y, grasp_z), yaw, bin_pose, opening)
                    descent_current = moveto.descent_current_a
                    if outcome != "placed":
                        attempted.append((x, y))
                    moveto.home()

                outcomes.append(outcome)
                log.write("picks", {
                    "time": time.strftime("%H:%M:%S"),
                    "snapshot": snapshot_no,
                    "attempt": attempt,
                    "class": cls,
                    "x": round(x, 1), "y": round(y, 1), "z": round(z, 1),
                    "top_z": "" if top_z is None else round(top_z, 1),
                    "grasp_x": round(grasp_x, 1), "grasp_y": round(grasp_y, 1),
                    "grasp_z": round(grasp_z, 1),
                    "width_mm": "" if width_mm is None else round(width_mm, 1),
                    "opening_mm": "" if opening is None else opening / 10,
                    "yaw_deg": "" if yaw is None else round(math.degrees(yaw)),
                    "outcome": outcome,
                    # 1 if it was picked although a neighbour stood in the way.
                    "crowded": int(crowded),
                    "failed_step": failed_step or "",
                    "grip_width_mm": "" if grip_width is None else round(grip_width, 1),
                    "descent_current_a": "" if descent_current is None else round(descent_current, 2),
                    "seconds": round(time.time() - started, 1),
                })

            if chosen is None or cli_args.hover:
                break

            if failed_step is not None:
                # The arm is not following commands; trying the rest would
                # only close the gripper on thin air.
                print(f"Move '{failed_step}' failed - stopping this snapshot's picks. "
                      "Check the arm (and the gripper, which may still hold a part).")
                break

            # The arm is home and out of view: look at the board again.
            if cli_args.no_hardware:
                targets = [t for t in targets if t[:2] != chosen[:2]]
            else:
                snapshot = node.detect_fresh()
                if snapshot is None:
                    print("Detection failed - stopping this snapshot's picks.")
                    break
                last_annotated = snapshot['annotated']
                targets = in_pick_area(snapshot['targets'])

        if not cli_args.no_hardware and outcomes:
            log.save_image(f"snapshot_{snapshot_no:02d}_after.jpg", last_annotated)

        row = {
            "time": time.strftime("%H:%M:%S"),
            "snapshot": snapshot_no,
            "target_class": cli_args.target_class or "all",
            "detected": detected,
            "located": located,
            "attempted": len(outcomes),
            "placed": outcomes.count("placed"),
            "empty_grasp": outcomes.count("empty_grasp"),
            "blocked_descent": outcomes.count("blocked_descent"),
            "no_free_yaw": outcomes.count("no_free_yaw"),
            "move_failed": outcomes.count("move_failed"),
            # Parts the last detection still saw on the board.
            "remaining": len(targets),
        }
        total_placed += row["placed"]
        total_located += row["located"]
        print(f"{trial} {snapshot_no}: {row['detected']} detected, {row['located']} located, "
              f"{row['placed']} placed, {row['empty_grasp']} empty grasp(s), "
              f"{row['blocked_descent']} blocked descent(s), "
              f"{row['no_free_yaw']} left (no free angle), {row['move_failed']} failed move, "
              f"{row['remaining']} still on the board.")

        answers = {}
        if not cli_args.no_hardware and not cli_args.hover:
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

        print(f"\n{snapshot_no} {trial.lower()}(s) done this session: "
              f"{total_placed} placed of {total_located} located in total.")
        print(f"Press 's' for {trial.lower()} {snapshot_no + 1}, 'q' to quit.")
        node.target_positions = None

    log.close()
    print(f"Run log saved in {log.dir}")

    if not cli_args.no_hardware:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
