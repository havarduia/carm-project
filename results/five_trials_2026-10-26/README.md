# Five trials, 2026-10-26

Run `logs/2026-10-26_182321`, code at commit `8362748`, model `yolo26s_v9_1280`. All five
trials were on the orange half of the board, with 4 capacitors, 4 resistors and 2 coils each.
A rosbag of the run (11 GB) is kept off the repo at `~/logs/bags/five_trials_2026-10-26_1824`.

| File | What it is |
| --- | --- |
| `kpi.pdf`, `.png` | IEEE double-column (7.16 in): per trial, parts placed, parts detected in the first image, and time per pick (`plot_kpi.py`) |
| `part_positions.pdf`, `.png` | All five trials in one xy plot, IEEE single-column (3.5 in): where the parts were, which were placed and which were never detected (`plot_positions.py`) |
| `grip_width.pdf`, `.png` | IEEE single-column: the width the gripper closed to on each placed part, by type (`plot_grip.py`) |
| `descent_current.pdf`, `.png` | IEEE single-column: the largest shoulder or elbow current change within 0.2 s on each descent, with a fitted normal distribution and the 1.2 A stop threshold (`plot_grip.py`) |
| `funnel.pdf`, `.png` | IEEE single-column: per part type, the parts on the board, detected, attempted and placed (`plot_detection.py`) |
| `confidence.pdf`, `.png` | IEEE single-column: per part, the detector's best own-class score over the looks before it was picked, against the 0.5 threshold (`plot_detection.py`). 49 of the 50 parts: a capacitor in trial 2 was out of the camera's view all trial and has no position |
| `part_confidence.csv` | Those scores: every look of the run (the frame before each `/yolo/detection_boxes` message in the rosbag) replayed through the model with no cut-off (`part_confidence.py`, needs ROS 2 and the repo venv) |
| `unpicked_positions.csv` | Robot-frame positions (mm) of the parts the model missed |
| `raw/picks.csv`, `raw/snapshots.csv` | `main.py`'s logs: one row per pick and one per trial, including the operator's answers |
| `raw/snapshot_*.jpg` | Per trial: the raw camera image, the detections, and the board afterwards |
