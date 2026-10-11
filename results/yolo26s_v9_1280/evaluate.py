"""Evaluate yolo26s_v9_1280 on its test split and write the numbers to data/.

Needs what git does not carry: the weights, the dataset and the training venv.
Run from the repo root:

    model_training/.venv/bin/python results/yolo26s_v9_1280/evaluate.py

make_plots.py then works from data/ alone.
"""
import csv
import json
import shutil
import tempfile
from collections import Counter
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
TRAINING = HERE.parents[1] / "model_training"
RUN = TRAINING / "runs" / "yolo26s_v9_1280"
DATASET = TRAINING / "datasets" / "combined_v9"
DATA = HERE / "data"

IMGSZ = 1280
ROBOT_CONF = 0.50  # CONF_THRESHOLD in detection_model/yolo_model.py


def write(name, header, rows):
    with open(DATA / name, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def main():
    DATA.mkdir(exist_ok=True)
    model = YOLO(RUN / "weights" / "best.pt")
    scratch = tempfile.mkdtemp()
    # Same call as the test evaluation at the end of model_training/train.py.
    # plots=True because Ultralytics only fills in the confusion matrix then;
    # its own plot files go to a scratch folder.
    common = dict(data=str(DATASET / "data.yaml"), split="test", imgsz=IMGSZ,
                  plots=True, verbose=False, project=scratch)

    # Curves and mAP over every confidence, as Ultralytics reports them.
    m = model.val(name="curves", **common)
    names = [m.names[i] for i in m.box.ap_class_index]
    (pr_x, pr_y, *_), (cf_x, f1, *_), (_, p, *_), (_, r, *_) = m.box.curves_results

    write("pr_curve.csv", ["recall"] + names,
          [[round(float(x), 4)] + [round(float(pr_y[c][i]), 4) for c in range(len(names))]
           for i, x in enumerate(pr_x)])
    write("confidence_curves.csv",
          ["confidence"] + [f"{kind}_{n}" for kind in ("precision", "recall", "f1") for n in names],
          [[round(float(x), 4)]
           + [round(float(curve[c][i]), 4) for curve in (p, r, f1) for c in range(len(names))]
           for i, x in enumerate(cf_x)])

    metrics = {"all": {"precision": float(m.box.mp), "recall": float(m.box.mr),
                       "map50": float(m.box.map50), "map50_95": float(m.box.map)},
               "per_class": {n: {"precision": float(m.box.p[i]), "recall": float(m.box.r[i]),
                                 "map50": float(m.box.ap50[i]), "map50_95": float(m.box.ap[i])}
                             for i, n in enumerate(names)}}
    (DATA / "test_metrics.json").write_text(json.dumps(metrics, indent=2))

    # What the robot acts on: only detections at or above its confidence threshold.
    m = model.val(name="robot", conf=ROBOT_CONF, **common)
    labels = [m.names[i] for i in sorted(m.names)] + ["background"]
    matrix = m.confusion_matrix.matrix  # rows: predicted, columns: labelled
    write("confusion_matrix_conf050.csv", ["predicted \\ labelled"] + labels,
          [[labels[i]] + [int(v) for v in row] for i, row in enumerate(matrix)])

    # Images and labelled objects per split and source.
    rows = []
    for split in ("train", "val", "test"):
        files = sorted((DATASET / split / "labels").glob("*.txt"))
        for source, mine in (("own photos", [f for f in files if f.name.startswith("carm_")]),
                             ("public PCB dataset", [f for f in files if f.name.startswith("pcb_")])):
            unique = [f for f in mine if not f.stem.endswith(("_r1", "_r2"))]
            counts = Counter()
            for f in unique:
                for line in f.read_text().splitlines():
                    if line.strip():
                        counts[m.names[int(line.split()[0])]] += 1
            rows.append([split, source, len(mine), len(unique)] + [counts[n] for n in labels[:-1]])
    write("dataset_counts.csv", ["split", "source", "image_files", "distinct_images"] + labels[:-1], rows)

    shutil.copy(RUN / "results.csv", DATA / "training_log.csv")
    shutil.copy(RUN / "args.yaml", DATA / "training_args.yaml")
    shutil.rmtree(scratch, ignore_errors=True)
    print("wrote", sorted(p.name for p in DATA.iterdir()))


if __name__ == "__main__":
    main()
