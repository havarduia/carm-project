"""The best the detector ever did on each part of the five trials of 2026-10-26.

Replays through yolo26s_v9_1280, with no confidence cut-off, the colour frames
of the run's rosbag that main.py detected on: the last frame before each of its
/yolo/detection_boxes messages (before every pick, and the check that ended the
trial). The score flickers from frame to frame, so other frames would give
scores main.py never saw. For every part it takes the highest score of a box
around the part's position, over the looks before the part was picked (all of
the trial's if it never was): for its own class, and for any class. main.py
only acts on boxes of CONF_THRESHOLD or more.

Part positions: the picks from raw/picks.csv (top-face centre), projected into
the image through the bag's camera calibration; the parts never picked at
their pixel in unpicked_positions.csv.

Writes part_confidence.csv. Needs ROS 2 sourced:
    python3 part_confidence.py ~/logs/bags/five_trials_2026-10-26_1824
"""
import csv
import datetime
import os
import sys

import cv2
import numpy as np
import rclpy.time
import rosbag2_py
import tf2_ros
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, ROOT)
from detection_model.yolo_model import CONF_THRESHOLD, MODEL_IMGSZ, MODEL_PATH  # noqa: E402

DAY = datetime.date(2026, 10, 26)
OFFSET_X, OFFSET_Y = -14, 10  # main.py: grasp point = top-face centre + offset
IMAGE = "/camera/camera/color/image_raw/compressed"
INFO = "/camera/camera/color/camera_info"
LOOK = "/yolo/detection_boxes"
NAME = {"transformer": "coil"}


def stamp(hms):
    return datetime.datetime.combine(DAY, datetime.time.fromisoformat(hms)).timestamp()


picks = list(csv.DictReader(open(os.path.join(HERE, "raw", "picks.csv"))))
unpicked = list(csv.DictReader(open(os.path.join(HERE, "unpicked_positions.csv"))))
for r in picks:
    # picks.csv times are whole seconds, at the end of the pick: the middle of
    # a pick is safely after the look it came from and before the next.
    r["end"] = stamp(r["time"])
    r["start"] = r["end"] - float(r["seconds"])
    r["middle"] = r["end"] - float(r["seconds"]) / 2
trials = sorted({int(r["snapshot"]) for r in picks})
span = {t: (min(r["start"] for r in picks if int(r["snapshot"]) == t) - 30,
            max(r["end"] for r in picks if int(r["snapshot"]) == t) + 15) for t in trials}

# Read the calibration and the frame of every look.
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
reader.set_filter(rosbag2_py.StorageFilter(topics=[IMAGE, INFO, LOOK, "/tf", "/tf_static"]))
buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
K, frames, latest = None, [], None
while reader.has_next():
    topic, data, t_ns = reader.read_next()
    t = t_ns / 1e9
    if topic in ("/tf", "/tf_static"):
        for tf in deserialize_message(data, get_message(types[topic])).transforms:
            if topic == "/tf_static":
                buffer.set_transform_static(tf, "bag")
            else:
                buffer.set_transform(tf, "bag")
    elif topic == INFO and K is None:
        K = np.array(deserialize_message(data, get_message(types[topic])).k).reshape(3, 3)
    elif topic == IMAGE:
        latest = data
    elif topic == LOOK and latest is not None:
        trial = next((n for n, (a, b) in span.items() if a <= t <= b), None)
        if trial is not None:
            msg = deserialize_message(latest, get_message(types[IMAGE]))
            frames.append((trial, t, cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)))

tf = buffer.lookup_transform("link_base", "camera_color_optical_frame", rclpy.time.Time()).transform
q, tr = tf.rotation, tf.translation
R = np.array([[1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
              [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2), 2 * (q.y * q.z - q.x * q.w)],
              [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x ** 2 + q.y ** 2)]])
camera = np.array([tr.x, tr.y, tr.z]) * 1000


def pixel(x, y, z):
    p = K @ (R.T @ (np.array([x, y, z]) - camera))
    return p[0] / p[2], p[1] / p[2]


# The parts: each pick of a new part, and each part never picked.
parts = []
for r in picks:
    t = int(r["snapshot"])
    x, y = float(r["grasp_x"]) - OFFSET_X, float(r["grasp_y"]) - OFFSET_Y
    if r["outcome"] != "placed" and any(p["trial"] == t and p["outcome"] != "placed"
                                        and abs(p["x"] - x) < 10 and abs(p["y"] - y) < 10 for p in parts):
        continue  # a second try at the same part
    u, v = pixel(x, y, float(r["top_z"]))
    parts.append({"trial": t, "part": NAME.get(r["class"], r["class"]), "outcome": r["outcome"],
                  "x": x, "y": y, "u": u, "v": v, "until": r["middle"]})
for r in unpicked:
    t = int(r["trial"])
    parts.append({"trial": t, "part": r["part"], "outcome": "not detected" if r["how"] != "hidden" else "hidden",
                  "x": float(r["x_mm"]), "y": float(r["y_mm"]), "u": float(r["u"]), "v": float(r["v"]),
                  "until": span[t][1]})

model = YOLO(MODEL_PATH)
for p in parts:
    p["conf_own"] = p["conf_any"] = 0.0
    p["any_class"] = ""
for trial, t, image in frames:
    result = model.predict(image, imgsz=MODEL_IMGSZ, conf=0.001, verbose=False)[0]
    boxes = list(zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(), result.boxes.conf.tolist()))
    for p in parts:
        if p["trial"] != trial or t > p["until"]:
            continue
        for (x1, y1, x2, y2), c, conf in boxes:
            if not (x1 <= p["u"] <= x2 and y1 <= p["v"] <= y2):
                continue
            name = NAME.get(result.names[int(c)], result.names[int(c)])
            if name == p["part"]:
                p["conf_own"] = max(p["conf_own"], conf)
            if conf > p["conf_any"]:
                p["conf_any"], p["any_class"] = conf, name

with open(os.path.join(HERE, "part_confidence.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["trial", "part", "outcome", "x_mm", "y_mm", "u", "v", "best_own_class", "best_any_class", "any_class"])
    for p in parts:
        w.writerow([p["trial"], p["part"], p["outcome"], round(p["x"], 1), round(p["y"], 1),
                    round(p["u"]), round(p["v"]), round(p["conf_own"], 3), round(p["conf_any"], 3), p["any_class"]])
print(f"{len(frames)} looks ({', '.join(str(sum(f[0] == t for f in frames)) for t in trials)} per trial), "
      f"{len(parts)} parts, threshold {CONF_THRESHOLD}")
