"""Plot a run's test-set KPIs (precision, recall, mAP): one panel per metric, one bar per class.

Reads runs/<NAME>/test_metrics.json, written by train.py after its final
test-split evaluation, and saves runs/<NAME>/kpi_test.png. Object counts next to
the class names are counted from the test labels of the dataset the run trained on.

    .venv/bin/python plot_kpi.py                      # yolo26s_v9_1280
    .venv/bin/python plot_kpi.py --name yolo26s_combined
"""
import argparse
import json
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

HERE = Path(__file__).resolve().parent

METRICS = [("precision", "Precision"), ("recall", "Recall"), ("map50", "mAP@50"), ("map50_95", "mAP@50-95")]
BAR, INK, MUTED, GRID = "#3d5a80", "#222222", "#6b6b6b", "#dddddd"


def test_counts(data_yaml):
    """(number of test images, {class name: labelled objects}) for a dataset yaml."""
    data = yaml.safe_load(data_yaml.read_text())
    names = data["names"]
    names = names if isinstance(names, dict) else dict(enumerate(names))
    labels = (Path(data.get("path", data_yaml.parent)) / data["test"]).parent / "labels"
    counts, n_images = Counter(), 0
    for label in labels.glob("*.txt"):
        n_images += 1
        for line in label.read_text().splitlines():
            if line.strip():
                counts[names[int(line.split()[0])]] += 1
    return n_images, counts


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--name", default="yolo26s_v9_1280", help="run name under runs/")
    args = parser.parse_args()
    run = HERE / "runs" / args.name

    metrics = json.loads((run / "test_metrics.json").read_text())
    train_args = yaml.safe_load((run / "args.yaml").read_text())
    data_yaml = Path(train_args["data"])
    n_images, counts = test_counts(data_yaml)

    groups = [("All classes", sum(counts.values()), metrics["all"])]
    groups += [(cls.capitalize(), counts[cls], vals) for cls, vals in metrics["per_class"].items()]

    # One panel per metric, one row per class: every bar is read against the
    # same 0-1 scale, so no legend or colour coding is needed.
    plt.rcParams.update({"font.family": ["Nimbus Sans", "Liberation Sans", "DejaVu Sans"], "font.size": 8})
    fig, axes = plt.subplots(1, len(METRICS), sharey=True, figsize=(7.2, 2.3), dpi=300)
    rows = range(len(groups))
    for ax, (key, label) in zip(axes, METRICS):
        values = [vals[key] for _, _, vals in groups]
        ax.barh(rows, values, height=0.62, color=BAR, zorder=3)
        for row, value in zip(rows, values):
            ax.text(value + 0.03, row, f"{value:.2f}", va="center", ha="left", color=INK, zorder=4)
        ax.set_title(label, loc="left", fontsize=8.5, color=INK, pad=6)
        ax.set_xlim(0, 1.18)
        ax.set_xticks([0, 0.5, 1.0])
        ax.set_xticklabels(["0", "0.5", "1"], color=MUTED)
        ax.tick_params(axis="x", length=0, pad=3)
        ax.tick_params(axis="y", length=0, pad=6)
        ax.grid(axis="x", color=GRID, linewidth=0.6, zorder=0)
        ax.set_axisbelow(True)
        for side in ("top", "right", "bottom"):
            ax.spines[side].set_visible(False)
        ax.spines["left"].set_color(INK)
        ax.spines["left"].set_linewidth(0.6)
    axes[0].set_yticks(list(rows))
    axes[0].set_yticklabels([f"{name} (n = {n})" for name, n, _ in groups], color=INK)
    axes[0].invert_yaxis()

    fig.text(0.008, 0.035,
             f"{args.name}. Test split of {data_yaml.parent.name}: {n_images} images, "
             f"{sum(counts.values())} labelled objects, inference size {train_args['imgsz']}.",
             fontsize=7, color=MUTED, ha="left")
    fig.subplots_adjust(left=0.2, right=0.995, top=0.88, bottom=0.22, wspace=0.12)

    out = run / "kpi_test.png"
    fig.savefig(out, facecolor="white")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
