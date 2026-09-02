"""Self-checks for the pure ordering and coordinate logic. Run: python3 test_main.py"""

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

from detection_model.yolo_model import deproject
from helpers.movement import MIN_Z_MM
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


def test_z_floor():
    # The floor is refused, not clamped, so a bad target fails loudly.
    arm = MockMoveArm()
    assert arm.move_to(200.0, 0.0, MIN_Z_MM) is True
    assert arm.move_to(200.0, 0.0, MIN_Z_MM - 0.1) is False


if __name__ == "__main__":
    test_order_targets()
    test_deproject()
    test_z_floor()
    print("ok")
