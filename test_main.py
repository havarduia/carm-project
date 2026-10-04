"""Self-checks for the pure ordering and coordinate logic. Run: python3 test_main.py"""

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
from helpers.grasp_yaw import choose_grasp_yaw, closing_axis
from helpers.movement import MIN_Z_MM, downward_orientation
from helpers.run_log import RunLog
from main.main import MockMoveArm, order_targets


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


def test_order_targets():
    # Visits the nearest remaining target each time, starting from the base origin.
    assert order_targets([(5, 0, 0), (1, 0, 0), (2, 0, 0)]) == [(1, 0, 0), (2, 0, 0), (5, 0, 0)]

    # Nearness is 3D, so a far-in-x but close-overall point wins.
    assert order_targets([(0, 0, 9), (3, 0, 0)])[0] == (3, 0, 0)

    # Every target is visited exactly once, and the input is left alone.
    targets = [(180.0, -25.0, 30.0), (210.0, 15.0, 32.0), (180.0, -25.0, 30.0)]
    original = list(targets)
    assert sorted(order_targets(targets)) == sorted(original)
    assert targets == original

    assert order_targets([]) == []

    # Class labels ride along untouched and do not affect the ordering.
    labelled = [(5, 0, 0, "resistor"), (1, 0, 0, "capacitor")]
    assert order_targets(labelled) == [(1, 0, 0, "capacitor"), (5, 0, 0, "resistor")]


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
    test_order_targets()
    test_deproject()
    test_bbox_points()
    test_bbox_extent()
    test_read_key()
    test_z_floor()
    test_grasp_yaw()
    test_downward_orientation()
    test_run_log()
    print("ok")
