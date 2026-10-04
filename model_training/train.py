"""Train YOLO26s on datasets/combined and score it on our held-out test images.

Run prepare_data.py first. Results land in runs/<NAME>/: weights/best.pt,
results.csv (per-epoch metrics), the PR / confusion-matrix plots, and
test_metrics.json from the final test-split evaluation.
"""
import argparse
import json
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--data", default="combined", help="dataset folder under datasets/")
    parser.add_argument("--name", default="yolo26s_combined", help="run name under runs/")
    # v8 export was stretched to 640 x 640; v9 is fit within 1280.
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    args = parser.parse_args()
    DATA = HERE / "datasets" / args.data / "data.yaml"
    NAME, IMGSZ = args.name, args.imgsz

    model = YOLO("yolo26s.pt")  # COCO-pretrained
    model.train(
        data=str(DATA),
        imgsz=IMGSZ,
        epochs=150,
        patience=30,
        batch=args.batch,
        seed=0,
        project=str(HERE / "runs"),
        name=NAME,
        exist_ok=True,
    )

    best = YOLO(HERE / "runs" / NAME / "weights" / "best.pt")
    metrics = best.val(data=str(DATA), split="test", imgsz=IMGSZ,
                       project=str(HERE / "runs"), name=f"{NAME}_test", exist_ok=True)

    names = best.names
    per_class = {
        names[c]: {
            "precision": float(metrics.box.p[i]),
            "recall": float(metrics.box.r[i]),
            "map50": float(metrics.box.ap50[i]),
            "map50_95": float(metrics.box.ap[i]),
        }
        for i, c in enumerate(metrics.box.ap_class_index)
    }
    result = {
        "all": {
            "precision": float(metrics.box.mp),
            "recall": float(metrics.box.mr),
            "map50": float(metrics.box.map50),
            "map50_95": float(metrics.box.map),
        },
        "per_class": per_class,
    }
    out = HERE / "runs" / NAME / "test_metrics.json"
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
