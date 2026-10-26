"""Self-checks for the pure ordering and coordinate logic. Run: python3 test_main.py"""

import math
import os
import sys

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

import numpy as np

from detection_model.yolo_model import (
    DEPTH_RANGE_M,
    MIN_BOX_SIZE_M,
    bbox_extent,
    bbox_points,
    deproject,
    read_key,
)
from helpers.grasp_yaw import choose_grasp_yaw, clamps_neighbour, closing_axis
from helpers.part_top import part_top
from helpers.segment import segment_top
from helpers.movement import (BLOCKED_CURRENT_A, GUARD_SETTLE_S, HOME_POSE_MM, MIN_Z_MM,
                              TorqueGuard, downward_orientation)
from helpers.run_log import RunLog
from main.main import (EMPTY_GRIP_MM, GRIPPER_OPEN, MAX_GRASP_OPEN, MIN_GRIPPER_OPEN, MAX_ATTEMPTS_PER_PART, PICK_AREA_MM, PLACE_BINS,
                       MockMoveArm, attempts_near, choose_grasp, finger_opening, in_pick_area,
                       pick_and_place, pick_order)


def _camera_model():
    """A plain 640x480 pinhole camera, no distortion."""
    info = CameraInfo()
    info.width, info.height = 640, 480
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [615.0, 0.0, 320.0, 0.0, 0.0, 615.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    model = PinholeCameraModel()
    model.fromCameraInfo(info)
    return model


def test_deproject():
    model = _camera_model()

    # A deprojected point must land at exactly the requested depth and project
    # back onto the pixel it came from. Scaling the unit ray by depth instead of
    # rescaling it fails this everywhere but the principal point.
    for u, v in [(320.0, 240.0), (600.0, 400.0), (40.0, 60.0), (639.0, 0.0)]:
        x, y, z = deproject(model, u, v, 0.5)
        assert abs(z - 0.5) < 1e-9, (u, v, z)
        back_u, back_v = model.project3dToPixel((x, y, z))
        assert abs(back_u - u) < 1e-6 and abs(back_v - v) < 1e-6, (u, v, back_u, back_v)

    # On the optical axis the ray is already (0, 0, 1), so nothing moves.
    assert deproject(model, 320.0, 240.0, 0.4) == (0.0, 0.0, 0.4)


def test_bbox_points():
    model = _camera_model()
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[238:243, 318:323] = 500  # 5x5 patch of 0.5 m readings

    pts = bbox_points(model, depth, {"x": 320.0, "y": 240.0, "width": 5, "height": 5})
    assert len(pts) == 25, len(pts)
    assert all(abs(z - 0.5) < 1e-9 for _, _, z in pts)

    # Unmeasured (0) and out-of-range pixels are dropped, not deprojected onto
    # the camera or out past the table.
    depth[240, 320] = 0
    depth[240, 321] = int(DEPTH_RANGE_M[1] * 1000) + 1
    assert len(bbox_points(model, depth, {"x": 320.0, "y": 240.0, "width": 5, "height": 5})) == 23

    # A box hanging off the frame edge is clipped, not wrapped or crashed.
    assert bbox_points(model, depth, {"x": 1.0, "y": 1.0, "width": 20, "height": 20}) == []


def test_bbox_extent():
    assert bbox_extent([]) is None

    centre, size = bbox_extent([(0.0, 0.0, 0.5), (0.1, 0.2, 0.6)])
    assert centre == [0.05, 0.1, 0.55]
    assert all(abs(a - b) < 1e-9 for a, b in zip(size, [0.1, 0.2, 0.1]))

    # A flat component measures ~0 deep; the box still has to be visible.
    _, size = bbox_extent([(0.0, 0.0, 0.5), (0.1, 0.1, 0.5)])
    assert size[2] == MIN_BOX_SIZE_M


def test_pick_order():
    home = HOME_POSE_MM[:2]
    near = (home[0] + 50.0, home[1], 20.0, "capacitor")
    far = (home[0] + 300.0, home[1], 22.0, "capacitor")
    tall = (home[0] + 200.0, home[1], 45.0, "capacitor")

    # Tallest first; parts of about the same height, nearest the home pose first.
    order = pick_order([(far, None), (near, None), (tall, None)])
    assert [t for t, _ in order] == [tall, near, far], order

    # The height that counts is the top face where it was found, not the
    # detector's point on the side wall.
    order = pick_order([(tall, None), (near, (near[0], near[1], 60.0))])
    assert order[0][0] == near, order

    assert pick_order([]) == []


def test_torque_guard():
    # A free descent: currents drift slowly as the arm's pose changes.
    guard = TorqueGuard()
    assert not any(guard.update(i / 100.0, [0.0, 0.4 + 0.001 * i, -2.9 - 0.001 * i, 0.0])
                   for i in range(200))
    assert guard.peak < BLOCKED_CURRENT_A / 2, guard.peak

    # Landing on something: the shoulder current climbs within a few samples.
    guard = TorqueGuard()
    tripped_at = None
    for i in range(200):
        push = max(0, i - 100) * BLOCKED_CURRENT_A / 5
        if guard.update(i / 100.0, [0.0, 0.4 + push, -2.9, 0.0]):
            tripped_at = i
            break
    assert tripped_at is not None and tripped_at - 100 < 10, tripped_at

    # The jolt of setting off does not count.
    guard = TorqueGuard()
    assert not guard.update(0.05, [0.0, 0.4, -2.9, 0.0])
    assert not guard.update(GUARD_SETTLE_S / 2, [0.0, 9.0, -2.9, 0.0])


def test_read_key():
    # Non-blocking: nothing typed yet means None, not a stalled node.
    r, w = os.pipe()
    stdin, original = os.fdopen(r), sys.stdin
    sys.stdin = stdin
    try:
        assert read_key() is None
        os.write(w, b"s")
        assert read_key() == "s"
    finally:
        sys.stdin = original
        stdin.close()
        os.close(w)


def test_z_floor():
    # The floor is refused, not clamped, so a bad target fails loudly.
    arm = MockMoveArm()
    assert arm.move_to(200.0, 0.0, MIN_Z_MM) is True
    assert arm.move_to(200.0, 0.0, MIN_Z_MM - 0.1) is False


def _floor_cloud():
    """A flat floor at z = 0, one point per mm, 200 mm square around the origin."""
    xs, ys = np.meshgrid(np.arange(-100.0, 100.0), np.arange(-100.0, 100.0))
    return np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1)


def _post(x, y, height):
    """A 10 mm square post standing on the floor."""
    xs, ys = np.meshgrid(np.arange(x - 5.0, x + 5.0), np.arange(y - 5.0, y + 5.0))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, height)], axis=1)


def test_grasp_yaw():
    import math

    floor = _floor_cloud()

    # Nothing around: the gripper is left unrotated.
    assert choose_grasp_yaw(floor, 0.0, 0.0, 6.0, 40.0, 0.0) == 0.0

    # At yaw 0 the fingers close along y, so a post 25 mm away in y sits under
    # a finger and the gripper has to turn away from it.
    cloud = np.vstack([floor, _post(0.0, 25.0, 30.0)])
    yaw = choose_grasp_yaw(cloud, 0.0, 0.0, 6.0, 40.0, 0.0)
    assert yaw is not None and abs(math.degrees(yaw)) >= 45, yaw

    # The same post 25 mm away in x is beside the fingers, not under them.
    cloud = np.vstack([floor, _post(25.0, 0.0, 30.0)])
    assert choose_grasp_yaw(cloud, 0.0, 0.0, 6.0, 40.0, 0.0) == 0.0

    # A post lower than the fingertips will go is no obstacle.
    cloud = np.vstack([floor, _post(0.0, 25.0, 2.0)])
    assert choose_grasp_yaw(cloud, 0.0, 0.0, 6.0, 40.0, 0.0) == 0.0

    # A camera that reads the whole scene 10 mm high must not see the floor
    # as an obstacle: heights are taken from the floor it sees.
    assert choose_grasp_yaw(floor + (0.0, 0.0, 10.0), 0.0, 0.0, 6.0, 40.0, 0.0) == 0.0

    # Ringed by posts on every side, and with no depth at all: no free yaw.
    ring = [_post(25.0 * math.cos(a), 25.0 * math.sin(a), 30.0)
            for a in np.arange(0.0, 2 * math.pi, math.pi / 8)]
    assert choose_grasp_yaw(np.vstack([floor] + ring), 0.0, 0.0, 6.0, 40.0, 0.0) is None
    # Unless any yaw will do: then the one with least in the way. Here the
    # ring has a post missing, so the fingers go where it was.
    gap = [p for i, p in enumerate(ring) if i not in (4, 12)]
    yaw = choose_grasp_yaw(np.vstack([floor] + gap), 0.0, 0.0, 6.0, 40.0, 0.0, best_effort=True)
    assert yaw is not None and abs(abs(math.degrees(yaw)) - 0) < 1, math.degrees(yaw)
    assert choose_grasp_yaw(np.zeros((0, 3)), 0.0, 0.0, 6.0, 40.0, 0.0) is None


def test_downward_orientation():
    import math

    def rotate(q, v):
        x, y, z, w = q
        u = np.array([x, y, z])
        return v + 2 * np.cross(u, np.cross(u, v) + w * v)

    for deg in (0, 30, -45, 90):
        q = downward_orientation(math.radians(deg))
        # The tool Z axis (the approach direction) always points straight down.
        assert np.allclose(rotate(q, np.array([0.0, 0.0, 1.0])), [0, 0, -1])
        # The tool Y axis is the one the fingers close along; it must match
        # what the free-space check assumes, up to sign.
        tool_y = rotate(q, np.array([0.0, 1.0, 0.0]))
        cx, cy = closing_axis(math.radians(deg))
        assert np.allclose(np.abs(tool_y[:2] @ (cx, cy)), 1.0) and abs(tool_y[2]) < 1e-9


def test_pick_area():
    (x_min, x_max), (y_min, y_max) = PICK_AREA_MM
    on_board = (380.0, 0.0, 15.0, "capacitor")
    assert in_pick_area([on_board]) == [on_board]
    # Parts lying in the bins are seen by the camera too, and must not be picked.
    for bx, by, bz in PLACE_BINS.values():
        assert in_pick_area([(bx, by, bz, "capacitor")]) == []
    assert in_pick_area([(x_max + 1, 0.0, 15.0, "capacitor"), (x_min, y_min - 1, 15.0, "x")]) == []


def test_attempts_near():
    tried = [(300.0, 50.0), (303.0, 52.0), (400.0, 50.0)]
    # A re-detection of the same part lands a few mm off, and still counts.
    assert attempts_near((301.0, 51.0, 12.0, "capacitor"), tried) == 2 == MAX_ATTEMPTS_PER_PART
    assert attempts_near((350.0, 50.0, 12.0, "capacitor"), tried) == 0
    # 2026-10-26: a big capacitor at (438, 14) was tried twice; the small one
    # against it at (425.6, 18.3) is within 15 mm of both tries, but they were
    # not its tries, so it must still be picked.
    big, small = (438.0, 13.0, 32.0, "capacitor"), (425.6, 18.3, 16.0, "capacitor")
    tried = [(438.0, 13.9), (439.8, 14.4)]
    assert attempts_near(small, tried) == 2
    assert attempts_near(small, tried, [big, small]) == 0
    assert attempts_near(big, tried, [big, small]) == 2


def test_empty_grasp():
    class Arm(MockMoveArm):
        """Closes to `closed_mm` and records what it was told to do."""

        def __init__(self, closed_mm, blocked_descent=False):
            self.closed_mm, self.calls = closed_mm, []
            self.blocked_descent = blocked_descent

        def set_gripper(self, pos):
            self.gripper_width_mm = self.closed_mm if pos == 0 else pos / 10.0
            self.calls.append(("grip", pos))
            return True

        def move_to(self, x, y, z, speed=None, yaw=0.0, guard=False):
            self.calls.append(("move", x, y))
            self.blocked = guard and self.blocked_descent
            return not self.blocked

    grasp, bin_pose = (300.0, 0.0, 20.0), (70.0, 220.0, 60.0)

    # A held part is carried to the bin.
    arm = Arm(closed_mm=10.4)
    assert pick_and_place(arm, grasp, 0.0, bin_pose) == ("placed", None, 10.4)
    assert ("move", 70.0, 220.0) in arm.calls

    # An empty close never travels to the bin, and leaves the gripper open.
    arm = Arm(closed_mm=EMPTY_GRIP_MM / 2)
    assert pick_and_place(arm, grasp, 0.0, bin_pose) == ("empty_grasp", None, EMPTY_GRIP_MM / 2)
    assert ("move", 70.0, 220.0) not in arm.calls
    assert arm.calls[-2][0] == "grip" and arm.calls[-2][1] > 0

    # A descent stopped by something under a finger: the arm lifts off again
    # without ever closing the gripper, and the part is left.
    arm = Arm(closed_mm=10.4, blocked_descent=True)
    assert pick_and_place(arm, grasp, 0.0, bin_pose) == ("blocked_descent", None, None)
    assert ("grip", 0) not in arm.calls and ("move", 70.0, 220.0) not in arm.calls
    assert arm.calls[-1] == ("move", 300.0, 0.0)


def test_finger_opening():
    # Unknown width: fully open, as before.
    assert finger_opening(None) == GRIPPER_OPEN
    # Small parts get the minimum, huge ones the maximum, and in between the
    # opening grows with the part.
    assert finger_opening(0.0) == MIN_GRIPPER_OPEN
    assert finger_opening(60.0) == MAX_GRASP_OPEN
    assert MIN_GRIPPER_OPEN < finger_opening(10.0) < finger_opening(15.0) < GRIPPER_OPEN

    floor = _floor_cloud()
    # Alone on the table, a part is taken at its own opening and with no turn.
    assert choose_grasp(floor, 0.0, 0.0, 6.0, 10.0) == (finger_opening(10.0), 0.0)
    assert choose_grasp(None, 0.0, 0.0, 6.0, 10.0) == (finger_opening(10.0), 0.0)

    # Ringed by posts 34 mm away, the full opening fits nowhere but a narrow one does.
    ring = [_post(34.0 * math.cos(a), 34.0 * math.sin(a), 30.0)
            for a in np.arange(0.0, 2 * math.pi, math.pi / 8)]
    cloud = np.vstack([floor] + ring)
    assert choose_grasp_yaw(cloud, 0.0, 0.0, 6.0, GRIPPER_OPEN / 10.0, 0.0) is None
    grasp = choose_grasp(cloud, 0.0, 0.0, 6.0, 8.0)
    assert grasp is not None and grasp[0] == finger_opening(8.0), grasp

    # A neighbour 18 mm to the side stands between the finger faces at yaw 0
    # and would be clamped along with the part, so the gripper turns or gives up.
    assert clamps_neighbour(0.0, 0.0, 30.0, 0.0, [(18.0, 0.0, 5.0)])
    assert not clamps_neighbour(0.0, 0.0, 30.0, 0.0, [(40.0, 0.0, 5.0)])
    assert not clamps_neighbour(0.0, 0.0, 30.0, 0.0, [])
    # A short neighbour is no obstacle to fingers that stop above its top:
    # a tall part is gripped over a short one standing right against it.
    short = [(13.0, 0.0, 4.5, 22.0)]
    assert clamps_neighbour(0.0, 0.0, 35.0, 0.0, short, tip_z=7.0)
    assert not clamps_neighbour(0.0, 0.0, 35.0, 0.0, short, tip_z=29.0)
    assert clamps_neighbour(0.0, 0.0, 35.0, 0.0, short, tip_z=23.0)
    yaw = choose_grasp_yaw(floor, 0.0, 0.0, 6.0, 30.0, 0.0, [(40.0, 0.0, 5.0)])
    assert yaw == 0.0, yaw
    assert choose_grasp(floor, 0.0, 0.0, 6.0, 10.0, [(18.0, 0.0, 5.0)]) is None

    # A tall part wider than the estimate blocks the narrow openings itself,
    # so the fingers open further until they clear it.
    wide = np.vstack([floor] + [_post(dx, dy, 30.0) for dx in (-10.0, 0.0, 10.0)
                                for dy in (-10.0, 0.0, 10.0)])
    opening, _ = choose_grasp(wide, 0.0, 0.0, 6.0, 8.0)
    assert opening > finger_opening(8.0), opening

    # A 40 mm wide part fills the strips of a 40 mm opening itself, at every
    # yaw; it is taken with the fingers open wider (no_free_yaw, 2026-10-14).
    block = np.vstack([floor] + [_post(dx, dy, 30.0) for dx in (-15.0, -5.0, 5.0, 15.0)
                                 for dy in (-15.0, -5.0, 5.0, 15.0)])
    assert choose_grasp_yaw(block, 0.0, 0.0, 6.0, GRIPPER_OPEN / 10.0, 0.0) is None
    grasp = choose_grasp(block, 0.0, 0.0, 6.0, 40.0)
    assert grasp is not None and GRIPPER_OPEN < grasp[0] <= MAX_GRASP_OPEN, grasp

    # A small part in the middle of that block is not opened past
    # GRIPPER_OPEN to clear it: fingers that wide close through the
    # neighbours (2026-10-26).
    assert choose_grasp(block, 0.0, 0.0, 6.0, 10.0) is None


def test_part_top():
    # A 20 mm wide, 30 mm tall block centred on (300, 0), seen by a camera
    # standing at (0, 0): its top face, and the side wall facing the camera.
    xs, ys = np.meshgrid(np.arange(290.0, 310.0), np.arange(-10.0, 10.0))
    top = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, 30.0)], axis=1)
    ys, zs = np.meshgrid(np.arange(-10.0, 10.0), np.arange(0.0, 30.0))
    wall = np.stack([np.full(ys.size, 290.0), ys.ravel(), zs.ravel()], axis=1)
    cloud = np.vstack([_floor_cloud() + (300.0, 0.0, 0.0), top, wall])

    # The detector's point is on that wall, two thirds of the way up. The
    # grasp belongs on the block's axis, at its full height.
    x, y, z = part_top(cloud, (290.0, 0.0, 20.0), 10.0, (0.0, 0.0))
    assert abs(x - 300.0) < 2.0 and abs(y) < 1.0 and abs(z - 30.0) < 0.5, (x, y, z)

    # A taller block right behind it must not be taken for its top.
    xs, ys = np.meshgrid(np.arange(314.0, 334.0), np.arange(-10.0, 10.0))
    behind = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, 50.0)], axis=1)
    x, y, z = part_top(np.vstack([cloud, behind]), (290.0, 0.0, 20.0), 10.0, (0.0, 0.0))
    assert abs(x - 300.0) < 2.0 and abs(z - 30.0) < 0.5, (x, y, z)

    # A thin part packed against a taller one (2026-10-26): the neighbour's
    # rim falls inside the search and is higher, so the search walks onto it.
    # That is not this part's top.
    # The box reads it 15 mm wide; it is 11.
    xs, ys = np.meshgrid(np.arange(290.0, 301.0), np.arange(-5.5, 5.5))
    thin = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, 18.0)], axis=1)
    xs, ys = np.meshgrid(np.arange(301.0, 327.0), np.arange(-13.0, 13.0))
    big = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, 40.0)], axis=1)
    floor = _floor_cloud() + (300.0, 0.0, 0.0)
    x, y, z = part_top(np.vstack([floor, thin]), (290.0, 0.0, 12.0), 7.5, (0.0, 0.0))
    assert abs(x - 295.5) < 1.5 and abs(z - 18.0) < 0.5, (x, y, z)
    assert part_top(np.vstack([floor, thin, big]), (290.0, 0.0, 12.0), 7.5, (0.0, 0.0)) is None

    # No depth over the part: nothing to go on.
    assert part_top(np.zeros((0, 3)), (290.0, 0.0, 20.0), 10.0, (0.0, 0.0)) is None
    assert part_top(_floor_cloud(), (290.0, 0.0, 20.0), 10.0, (0.0, 0.0)) is None


def test_segment_top():
    # Seen from straight above at 0.5 mm per pixel, about what the camera
    # gets at the board: pixel (u, v) is at x = u / 2, y = v / 2. A thin part,
    # 11 mm wide and 18 tall, stands against a 26 mm wide one 40 tall, as on
    # 2026-10-26; the board is at z = 2.
    grid = np.zeros((240, 240, 3), dtype=np.float32)
    grid[..., 1], grid[..., 0] = np.indices((240, 240)) / 2.0
    grid[..., 2] = 2.0
    grid[90:112, 80:102, 2] = 18.0
    grid[76:128, 102:154, 2] = 40.0
    # Its box reaches well into the big one, as boxes in a packed group do.
    x, y, z = segment_top(grid, (72, 82, 120, 120))
    assert abs(x - 45.25) < 1.0 and abs(y - 50.25) < 1.0 and abs(z - 18.0) < 0.5, (x, y, z)
    # The big one cut out of its own box, with the thin one in it.
    x, y, z = segment_top(grid, (72, 68, 160, 136))
    assert abs(x - 64.25) < 1.0 and abs(y - 51.25) < 1.0 and abs(z - 40.0) < 0.5, (x, y, z)
    # No depth readings, or nothing standing in the box: no part.
    assert segment_top(np.full_like(grid, np.nan), (72, 82, 120, 120)) is None
    assert segment_top(grid, (180, 180, 230, 230)) is None


def test_run_log():
    import csv
    import tempfile

    with tempfile.TemporaryDirectory() as root:
        log = RunLog(root)
        log.write("picks", {"attempt": 1, "outcome": "placed"})
        log.write("picks", {"attempt": 2, "outcome": "no_free_yaw"})
        # Rows are on disk as soon as they are written, not only at close().
        with open(os.path.join(log.dir, "picks.csv")) as f:
            rows = list(csv.DictReader(f))
        assert rows == [{"attempt": "1", "outcome": "placed"},
                        {"attempt": "2", "outcome": "no_free_yaw"}], rows
        log.close()


if __name__ == "__main__":
    test_pick_order()
    test_torque_guard()
    test_deproject()
    test_bbox_points()
    test_bbox_extent()
    test_read_key()
    test_z_floor()
    test_grasp_yaw()
    test_downward_orientation()
    test_pick_area()
    test_attempts_near()
    test_empty_grasp()
    test_finger_opening()
    test_part_top()
    test_segment_top()
    test_run_log()
    print("ok")
