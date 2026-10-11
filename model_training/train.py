"""Train YOLO26s on datasets/combined_v9 and score it on our held-out test images.

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
    parser.add_argument("--data", default="combined_v9", help="dataset folder under datasets/")
    parser.add_argument("--name", default="yolo26s_v9_1280", help="run name under runs/")
    # Our photos are exported fit within 1280 px. Batch 8 fills a 12 GB card at that size.
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--model", default="yolo26s.pt",
                        help="weights, or a model yaml (e.g. yolo26s-p2.yaml) started from yolo26s.pt")
    args = parser.parse_args()
    DATA = HERE / "datasets" / args.data / "data.yaml"
    NAME, IMGSZ = args.name, args.imgsz

    model = YOLO(args.model)  # COCO-pretrained
    if args.model.endswith(".yaml"):
        model.load("yolo26s.pt")  # copies every layer the two architectures share
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
