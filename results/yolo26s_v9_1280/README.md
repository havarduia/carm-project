# yolo26s_v9_1280: the detector used in the trials

Plots and numbers for the model the robot ran during the five trials in
`../five_trials_2026-10-11/`. What the model is, how it was trained and its known
limitations are described in `model_training/yolo26s_v9_1280.md`.

Everything here is measured on the model's own test split: 15 of our own photos with
290 labelled objects, at inference size 1280. These are not the trial images. The
trials ran with the camera at 1920x1080 and sharpness 100, a setting the model was
never trained or tested on; how it did there is in the trial folder.

## Result on the test split

| Class | Objects | Precision | Recall | mAP@50 | mAP@50-95 |
| --- | --- | --- | --- | --- | --- |
| All | 290 | 0.78 | 0.85 | 0.84 | 0.60 |
| Capacitor | 253 | 0.85 | 0.88 | 0.91 | 0.70 |
| Resistor | 8 | 0.76 | 0.88 | 0.81 | 0.49 |
| Transformer | 29 | 0.74 | 0.78 | 0.80 | 0.60 |

At the robot's confidence threshold of 0.50: 254 of the 290 labelled parts were
reported with the right class, 1 with the wrong class, 35 were not reported, and 47
detections had no label under them.

## Plots

| File | Shows |
| --- | --- |
| `plots/01_test_metrics.png` | Precision, recall and mAP per class |
| `plots/02_training_curves.png` | Validation score and losses per epoch, with the saved epoch marked |
| `plots/03_precision_recall.png` | Precision against recall, per class |
| `plots/04_confidence_curves.png` | Precision, recall and F1 against the confidence threshold, with the robot's 0.50 marked |
| `plots/05_confusion_matrix.png` | What the model said against what was labelled, at confidence 0.50 |
| `plots/06_dataset.png` | Images and labelled objects per split and source |

## Data and scripts

| Path | Contents |
| --- | --- |
| `data/test_metrics.json` | The table above, unrounded; identical to the file written at the end of training |
| `data/pr_curve.csv` | Precision at 1,000 recall levels, per class |
| `data/confidence_curves.csv` | Precision, recall and F1 at 1,000 confidence thresholds, per class |
| `data/confusion_matrix_conf050.csv` | Counts behind the confusion matrix |
| `data/dataset_counts.csv` | Images and labelled objects per split and source |
| `data/training_log.csv`, `data/training_args.yaml` | Per-epoch log and settings of the training run |
| `evaluate.py` | Writes `data/`. Needs the weights, the dataset and the training venv, none of which are in git |
| `make_plots.py` | Draws `plots/` from `data/` alone: `python3 make_plots.py` |

## Reading these numbers

- The labels are incomplete. A review on 2026-10-10 found 115 real parts missing from
  the labels of our 152 photos, so some of the 47 "extra" detections are correct.
- The test photos show the same boards as the training photos. The scores say how well
  the model knows these boards, not how it does on a new one.
- The resistor numbers rest on 8 objects.
