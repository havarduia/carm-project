# Five pick-and-place trials, 2026-10-11

Everything recorded for the five trials that count as the result, plus the data and
plots derived from them. One run of `main/main.py` on commit `4f50dbe` of `main`,
22:28 to 22:44, log folder `logs/2026-10-11_222859/`.

## Result

| | |
| --- | --- |
| Trials | 5, ten loose capacitors on the board each |
| Parts picked and binned | 50 of 50 |
| Retries, empty grasps, blocked descents, parts left | 0 |
| Operator counts (missed, false, dropped, wrong bin, knocked) | all 0 |
| Time per pick | 12.4 s mean (11.8 to 13.9 s) |
| Time to clear a board of ten | 125 to 130 s |
| Detections in the first image of each trial | 10, 10, 11, 12, 10 (3 double boxes, one labelled transformer) |
| Picks made with a neighbour in the way | 2 |

Settings that matter for reading the numbers: detector `yolo26s_v9_1280` at confidence
0.50, camera at 1920x1080 with sharpness 100, travel speed 0.6 and descent speed 0.3,
finger clearance 6 mm per side, camera-to-arm trim x -14 mm and y +10 mm.

## What is in this folder

| Path | Contents |
| --- | --- |
| `raw/picks.csv` | One row per pick (50), as logged by `main.py` |
| `raw/snapshots.csv` | One row per trial, with the operator's answers |
| `raw/terminal.txt` | The complete terminal output of the run |
| `raw/snapshot_NN_raw.jpg`, `_detections.jpg`, `_after.jpg` | Camera frame at the start of each trial, the same with boxes, and the last detection of the trial |
| `raw/kpi_trials.png` | KPI figure from `plot_trials.py` |
| `data/trial_summary.csv` | Per trial: parts, placed, duration, mean time per pick, highest descent current |
| `data/pick_phase_times.csv` | Per pick: seconds for each of the ten arm and gripper steps, from the terminal timestamps |
| `data/detection_passes.csv` | Per detection pass (55): detections by class, and how many were dropped as duplicates or for lack of depth |
| `plots/` | The figures listed below |
| `make_plots.py` | Rebuilds `data/` and `plots/` from `raw/`: `python3 make_plots.py` |

## Plots

| File | Shows |
| --- | --- |
| `01_kpi_trials.png` | Per trial: parts picked, time per pick, detections |
| `02_time_per_phase.png` | Mean seconds for each step of a pick |
| `03_time_per_pick.png` | Seconds for each of the 50 picks, in order |
| `04_pick_positions.png` | Where on the board the parts were gripped |
| `05_part_sizes.png` | Part widths (gripper) and heights (camera) |
| `06_width_estimate_and_opening.png` | Width from the detection box against the measured width, and the finger opening used |
| `07_gripper_angle.png` | Gripper rotation chosen for each pick |
| `08_descent_current.png` | Motor-current jump of each descent against the stop threshold |
| `09_detections_between_picks.png` | Detections against parts left, after every pick |
| `10_grasp_height.png` | Fingertip height against part height |
| `11_time_to_clear_board.png` | Seconds from first move to last return home, per trial |

## Limits of this record

- The camera frames between picks, the depth images and the arm's joint motion were
  not saved. Detections between picks are known only as the counts in the terminal.
- Heights come from the camera, which reads 2 to 7 mm low depending on where the part
  stands. Widths in `picks.csv` are the gripper opening after closing.
- All 50 parts were capacitors, so the trials do not test sorting into different bins.
- `detection_passes.csv` cannot say where an extra box was. In the first image of each
  trial the extras were double boxes on one part; later ones may be parts in the bin,
  which is in the camera's view.
