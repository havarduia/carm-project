"""Live RealSense preview: SPACE saves the current colour frame, Q or Esc quits.

Start the RealSense driver first (1920x1080 colour, rgb_camera.sharpness 100).
Run with the ROS python, not the training venv:

    python3 capture_frames.py                 # -> datasets/realsense_raw/
    python3 capture_frames.py --out some/dir

Frames are saved full size as frame_0001.png, frame_0002.png, ...; numbering
continues from whatever is already in the folder.
"""
import argparse
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

HERE = Path(__file__).resolve().parent
TOPIC = "/camera/camera/color/image_raw"
EXPECTED_SIZE = (1920, 1080)
PREVIEW_WIDTH = 1280
WINDOW = "RealSense capture  [SPACE = save, Q = quit]"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(HERE / "datasets" / "realsense_raw"))
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    saved = sorted(out.glob("frame_*.png"))
    next_index = int(saved[-1].stem.split("_")[1]) + 1 if saved else 1

    rclpy.init()
    node = Node("capture_frames")
    bridge = CvBridge()
    latest = {}

    def on_image(msg):
        latest["frame"] = bridge.imgmsg_to_cv2(msg, "bgr8")

    node.create_subscription(Image, TOPIC, on_image, 2)
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    flash_until = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            frame = latest.get("frame")
            if frame is None:
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            h, w = frame.shape[:2]
            scale = PREVIEW_WIDTH / w
            view = cv2.resize(frame, (PREVIEW_WIDTH, int(h * scale)), interpolation=cv2.INTER_AREA)
            text = f"saved: {next_index - 1}   {w}x{h}   SPACE = save, Q = quit"
            colour = (0, 255, 0)
            if (w, h) != EXPECTED_SIZE:
                text += f"   WARNING: expected {EXPECTED_SIZE[0]}x{EXPECTED_SIZE[1]}"
                colour = (0, 0, 255)
            cv2.rectangle(view, (0, 0), (PREVIEW_WIDTH, 34), (0, 0, 0), -1)
            cv2.putText(view, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2, cv2.LINE_AA)
            if time.time() < flash_until:
                cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1), (0, 255, 0), 12)
            cv2.imshow(WINDOW, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                path = out / f"frame_{next_index:04d}.png"
                cv2.imwrite(str(path), frame)
                print(f"saved {path}", flush=True)
                next_index += 1
                flash_until = time.time() + 0.4
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    print(f"{next_index - 1} frames in {out}")


if __name__ == "__main__":
    main()
