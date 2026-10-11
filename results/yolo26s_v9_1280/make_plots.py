"""Draw the plots for yolo26s_v9_1280 from the numbers in data/.

    python3 make_plots.py

data/ is written by evaluate.py, which needs the weights and the dataset;
this script needs neither.
"""
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
DATA, PLOTS = HERE / "data", HERE / "plots"

BAR, LIGHT, INK, MUTED, GRID = "#3d5a80", "#b9c4d0", "#222222", "#6b6b6b", "#dddddd"
ACCENT = "#c0504d"  # the robot's threshold and other reference lines
# Three series that have to be told apart in one panel.
SERIES = {"precision": "#2a78d6", "recall": "#eb6834", "f1": "#1baf7a"}

CLASSES = ["capacitor", "resistor", "transformer"]
ROBOT_CONF = 0.50


def read(name):
    with open(DATA / name, newline="") as f:
        return list(csv.DictReader(f))


def base(ax, title=None, grid="y"):
    if title:
        ax.set_title(title, loc="left", fontsize=8.5, color=INK, pad=6)
    ax.tick_params(length=0, pad=3, colors=MUTED)
    ax.grid(axis=grid, color=GRID, linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
        ax.spines[side].set_linewidth(0.6)


def save(fig, name, note=None):
    if note:
        fig.text(0.008, 0.012, note, fontsize=6.5, color=MUTED, ha="left", va="bottom", linespacing=1.5)
    fig.savefig(PLOTS / name, facecolor="white")
    plt.close(fig)
    print("saved plots/" + name)


def test_counts():
    row = next(r for r in read("dataset_counts.csv") if r["split"] == "test" and r["source"] == "own photos")
    return {c: int(row[c]) for c in CLASSES}, int(row["distinct_images"])


def plot_test_metrics():
    metrics = json.loads((DATA / "test_metrics.json").read_text())
    counts, n_images = test_counts()
    groups = [("All classes", sum(counts.values()), metrics["all"])]
    groups += [(c.capitalize(), counts[c], metrics["per_class"][c]) for c in CLASSES]
    keys = [("precision", "Precision"), ("recall", "Recall"), ("map50", "mAP@50"), ("map50_95", "mAP@50-95")]

    fig, axes = plt.subplots(1, len(keys), sharey=True, figsize=(7.2, 2.4), dpi=300)
    rows = range(len(groups))
    for ax, (key, label) in zip(axes, keys):
        values = [g[2][key] for g in groups]
        ax.barh(rows, values, height=0.62, color=BAR, zorder=3)
        for row, value in zip(rows, values):
            ax.text(value + 0.03, row, f"{value:.2f}", va="center", ha="left", color=INK)
        ax.set_xlim(0, 1.18)
        ax.set_xticks([0, 0.5, 1.0])
        ax.set_xticklabels(["0", "0.5", "1"])
        base(ax, label, grid="x")
        ax.spines["bottom"].set_visible(False)
    axes[0].set_yticks(list(rows))
    axes[0].set_yticklabels([f"{name} (n = {n})" for name, n, _ in groups], color=INK)
    axes[0].invert_yaxis()
    fig.subplots_adjust(left=0.2, right=0.995, top=0.88, bottom=0.27, wspace=0.12)
    save(fig, "01_test_metrics.png",
         f"Test split: {n_images} of our own photos, {sum(counts.values())} labelled objects, inference size 1280. "
         f"Precision and recall are at the confidence with the best F1.\nThe resistor row rests on "
         f"{counts['resistor']} objects.")


def plot_training():
    log = read("training_log.csv")
    log = [{k.strip(): float(v) for k, v in row.items()} for row in log]
    epochs = [r["epoch"] for r in log]
    best = max(log, key=lambda r: 0.1 * r["metrics/mAP50(B)"] + 0.9 * r["metrics/mAP50-95(B)"])

    fig, (a, b, c) = plt.subplots(1, 3, figsize=(7.2, 2.6), dpi=300)
    for key, label in (("metrics/mAP50(B)", "mAP@50"), ("metrics/mAP50-95(B)", "mAP@50-95")):
        values = [r[key] for r in log]
        a.plot(epochs, values, color=BAR, linewidth=1.0, zorder=3)
        a.text(epochs[-1] + 2, values[-1], label, va="center", color=INK, fontsize=7)
    a.axvline(best["epoch"], color=ACCENT, linewidth=0.8, zorder=2)
    a.text(best["epoch"] - 2, 0.06, f"saved: epoch {best['epoch']:.0f}", color=ACCENT, ha="right", fontsize=7)
    a.set_ylim(0, 1)
    a.set_xlim(0, epochs[-1] + 38)
    base(a, "Score on the validation photos")

    for ax, key, title in ((b, "box_loss", "Box loss"), (c, "cls_loss", "Class loss")):
        for split, colour in (("train", MUTED), ("val", BAR)):
            values = [r[f"{split}/{key}"] for r in log]
            ax.plot(epochs, values, color=colour, linewidth=1.0, zorder=3)
            ax.text(epochs[-1] + 2, values[-1], "training" if split == "train" else "validation",
                    va="center", color=colour, fontsize=7)
        ax.set_xlim(0, epochs[-1] + 40)
        ax.set_ylim(0, 1.5)
        base(ax, title)
    for ax in (a, b, c):
        ax.set_xlabel("Epoch", color=MUTED)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.88, bottom=0.3, wspace=0.26)
    save(fig, "02_training_curves.png",
         f"{len(log)} epochs were run before early stopping (no improvement for 30 epochs). "
         f"The weights in use are from epoch {best['epoch']:.0f}.\n"
         f"The training class loss of the first epoch ({log[0]['train/cls_loss']:.1f}) is above the range shown.")


def plot_pr():
    rows = read("pr_curve.csv")
    metrics = json.loads((DATA / "test_metrics.json").read_text())["per_class"]
    counts, _ = test_counts()
    recall = [float(r["recall"]) for r in rows]
    fig, axes = plt.subplots(1, 3, sharey=True, figsize=(7.2, 2.7), dpi=300)
    for ax, cls in zip(axes, CLASSES):
        precision = [float(r[cls]) for r in rows]
        ax.fill_between(recall, precision, color=LIGHT, alpha=0.45, linewidth=0, zorder=2)
        ax.plot(recall, precision, color=BAR, linewidth=1.2, zorder=3)
        ax.text(0.04, 0.08, f"area (AP@50) = {metrics[cls]['map50']:.2f}", color=INK, fontsize=7.5)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1.03)
        ax.set_xticks([0, 0.5, 1.0])
        ax.set_xticklabels(["0", "0.5", "1"])
        ax.set_xlabel("Recall", color=MUTED)
        base(ax, f"{cls.capitalize()} (n = {counts[cls]})", grid="both")
    axes[0].set_ylabel("Precision", color=MUTED)
    fig.subplots_adjust(left=0.07, right=0.99, top=0.88, bottom=0.3, wspace=0.1)
    save(fig, "03_precision_recall.png",
         "Precision against recall as the confidence threshold is lowered, on the test photos, boxes matched at "
         "IoU 0.5.\nThe steps in the resistor curve are single objects: there are only eight.")


def plot_confidence():
    rows = read("confidence_curves.csv")
    conf = [float(r["confidence"]) for r in rows]
    fig, axes = plt.subplots(1, 3, sharey=True, figsize=(7.2, 2.8), dpi=300)
    for ax, cls in zip(axes, CLASSES):
        for kind, colour in SERIES.items():
            ax.plot(conf, [float(r[f"{kind}_{cls}"]) for r in rows], color=colour, linewidth=1.1, zorder=3,
                    label={"precision": "Precision", "recall": "Recall", "f1": "F1"}[kind])
        ax.axvline(ROBOT_CONF, color=ACCENT, linewidth=0.8, zorder=2)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1.03)
        ax.set_xticks([0, 0.5, 1.0])
        ax.set_xticklabels(["0", "0.5", "1"])
        ax.set_xlabel("Confidence threshold", color=MUTED)
        base(ax, cls.capitalize(), grid="both")
    axes[0].text(ROBOT_CONF + 0.03, 0.05, "robot: 0.50", color=ACCENT, fontsize=7)
    axes[0].legend(loc="lower left", frameon=False, fontsize=7, handlelength=1.4, labelcolor=INK,
                   bbox_to_anchor=(0.0, 0.12))
    fig.subplots_adjust(left=0.06, right=0.99, top=0.88, bottom=0.27, wspace=0.1)
    at = min(range(len(conf)), key=lambda i: abs(conf[i] - ROBOT_CONF))
    parts = ", ".join(f"{c} P {float(rows[at]['precision_' + c]):.2f} / R {float(rows[at]['recall_' + c]):.2f}"
                      for c in CLASSES)
    save(fig, "04_confidence_curves.png",
         f"What raising or lowering the confidence threshold would do. At the robot's 0.50: {parts}.")


def plot_confusion():
    rows = read("confusion_matrix_conf050.csv")
    labels = [k for k in rows[0] if not k.startswith("predicted")]
    matrix = np.array([[int(r[label]) for label in labels] for r in rows])
    shown = np.ma.masked_equal(matrix, 0)

    fig, ax = plt.subplots(figsize=(5.0, 3.7), dpi=300)
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("one_hue", ["#eef2f6", BAR])
    ax.imshow(shown, cmap=cmap, vmin=0, vmax=matrix.max(), zorder=2)
    for i in range(len(labels)):
        for j in range(len(labels)):
            value = matrix[i, j]
            if i == j == len(labels) - 1:
                continue
            ax.text(j, i, str(value), ha="center", va="center", fontsize=8.5,
                    color="white" if value > matrix.max() * 0.5 else (INK if value else MUTED))
    shown_names = [name.capitalize() if name != "background" else "Nothing" for name in labels]
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(shown_names, fontsize=7.5)
    ax.set_yticks(range(len(labels)))
    ax.set_yticklabels(shown_names)
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position("top")
    ax.set_xlabel("Labelled as", color=MUTED, labelpad=6)
    ax.set_ylabel("Model said", color=MUTED)
    ax.tick_params(length=0, colors=INK)
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.subplots_adjust(left=0.2, right=0.97, top=0.82, bottom=0.2)
    extra = matrix[:-1, -1].sum()
    missed = matrix[-1, :-1].sum()
    save(fig, "05_confusion_matrix.png",
         f"Test photos, detections with confidence 0.50 or more, as the robot uses them. Bottom row: "
         f"{missed} labelled parts\nthe model did not report. Right column: {extra} detections with no label "
         f"under them; the labels are known\nto be incomplete, so part of these are real parts nobody labelled.")


def plot_dataset():
    rows = read("dataset_counts.csv")
    groups = [("Training, own photos", "train", "own photos"),
              ("Training, public dataset", "train", "public PCB dataset"),
              ("Validation, own photos", "val", "own photos"),
              ("Test, own photos", "test", "own photos")]
    picked = [next(r for r in rows if r["split"] == split and r["source"] == source) for _, split, source in groups]
    panels = [("Images", [int(r["distinct_images"]) for r in picked])]
    panels += [(f"{c.capitalize()}s labelled", [int(r[c]) for r in picked]) for c in CLASSES]

    fig, axes = plt.subplots(1, len(panels), sharey=True, figsize=(7.2, 2.3), dpi=300)
    ys = range(len(groups))
    for ax, (title, values) in zip(axes, panels):
        ax.barh(ys, values, height=0.62, color=BAR, zorder=3)
        for y, value in zip(ys, values):
            ax.text(value + max(values) * 0.04, y, f"{value:,}", va="center", ha="left", color=INK)
        ax.set_xlim(0, max(values) * 1.45)
        ax.set_xticks([])
        base(ax, title, grid="x")
        ax.spines["bottom"].set_visible(False)
    axes[0].set_yticks(list(ys))
    axes[0].set_yticklabels([name for name, _, _ in groups], color=INK)
    axes[0].invert_yaxis()
    fig.subplots_adjust(left=0.235, right=0.995, top=0.86, bottom=0.25, wspace=0.1)
    repeats = int(picked[0]["image_files"]) // max(int(picked[0]["distinct_images"]), 1)
    save(fig, "06_dataset.png",
         f"Each of our own training photos is included {repeats} times, so that the public images do not drown "
         f"them out; the counts here are of distinct photos.\nNo photo is in more than one split. "
         f"Each panel has its own scale.")


def main():
    PLOTS.mkdir(exist_ok=True)
    plt.rcParams.update({"font.family": ["Nimbus Sans", "Liberation Sans", "DejaVu Sans"], "font.size": 8})
    plot_test_metrics()
    plot_training()
    plot_pr()
    plot_confidence()
    plot_confusion()
    plot_dataset()


if __name__ == "__main__":
    main()
