"""Two IEEE single-column figures (3.5 in wide) on what the detector found in the
five trials of 2026-10-26, with no title or caption text:

funnel:     per part type, the parts on the board, detected (a box of their own
            class at CONF_THRESHOLD or more in at least one look), attempted
            and placed in the right bin.
confidence: per part, the detector's best own-class score over the looks
            before it was picked, from part_confidence.csv (part_confidence.py).

Writes funnel.pdf/.png and confidence.pdf/.png.
"""
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONF_THRESHOLD = 0.5  # detection_model/yolo_model.py
TYPES = ("capacitor", "coil", "resistor")
LABEL = {"capacitor": "Capacitor", "coil": "Coil", "resistor": "Resistor"}
COLOR = {"capacitor": "#2a78d6", "coil": "#eb6834", "resistor": "#1baf7a"}
INK, GRID = "#2b2b2a", "#e6e5e1"

parts = list(csv.DictReader(open(os.path.join(HERE, "part_confidence.csv"))))
trials = list(csv.DictReader(open(os.path.join(HERE, "raw", "snapshots.csv"))))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Nimbus Roman", "Times New Roman", "DejaVu Serif"],
                     "font.size": 7, "pdf.fonttype": 42})


def style(ax):
    ax.spines[["top", "right"]].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK)
        ax.spines[side].set_linewidth(0.5)
    ax.tick_params(colors=INK, labelsize=7, length=2, width=0.5)
    ax.set_axisbelow(True)


def save(fig, name):
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(HERE, f"{name}.{ext}"), dpi=300, facecolor="white")


# Funnel: grouped bars, one group per stage.
on_board = {p: sum(int(t[f"on_board_{'transformer' if p == 'coil' else p}"]) for t in trials) for p in TYPES}
stages = {
    "On board": on_board,
    "Detected": {p: sum(r["part"] == p and float(r["best_own_class"]) >= CONF_THRESHOLD for r in parts) for p in TYPES},
    "Attempted": {p: sum(r["part"] == p and r["outcome"] in ("placed", "empty_grasp") for r in parts) for p in TYPES},
    "Placed": {p: sum(r["part"] == p and r["outcome"] == "placed" for r in parts) for p in TYPES},
}
fig, ax = plt.subplots(figsize=(3.5, 1.9))
width = 0.26
for i, p in enumerate(TYPES):
    x = np.arange(len(stages)) + (i - 1) * width
    values = [stages[s][p] for s in stages]
    ax.bar(x, values, width=width, color=COLOR[p], edgecolor="white", linewidth=0.5, label=LABEL[p], zorder=3)
    for xi, v in zip(x, values):
        ax.text(xi, v + 0.3, str(v), ha="center", va="bottom", color=INK, fontsize=6.5)
ax.set_xticks(range(len(stages)))
ax.set_xticklabels(stages, color=INK)
ax.set_ylim(0, 23)
ax.set_ylabel("Parts", color=INK, labelpad=2)
ax.grid(axis="y", color=GRID, linewidth=0.5)
ax.legend(frameon=False, fontsize=7, labelcolor=INK, ncol=3, loc="upper right", handlelength=1.2)
style(ax)
fig.subplots_adjust(left=0.11, right=0.98, top=0.97, bottom=0.12)
save(fig, "funnel")

# Confidence: one row per type, one dot per part.
fig, ax = plt.subplots(figsize=(3.5, 1.6))
jitter = np.random.default_rng(1)
for row, p in enumerate(TYPES):
    # Parts the detector drew no box of their class on at all are one marker,
    # labelled with the count; the rest one jittered dot each.
    mine = [r for r in parts if r["part"] == p]
    none = [r for r in mine if float(r["best_own_class"]) == 0.0]
    if len(none) > 1:
        ax.annotate(f"{len(none)}×", (0.0, row), xytext=(5, 0), textcoords="offset points",
                    va="center", fontsize=6.5, color=INK)
    for r in [r for r in mine if r not in none] + none[:1]:
        score = float(r["best_own_class"])
        y = row if r in none else row + jitter.uniform(-0.22, 0.22)
        if r["outcome"] == "placed":
            ax.scatter(score, y, s=14, color=COLOR[p], edgecolor="white", linewidth=0.5, zorder=3)
        elif r["outcome"] == "empty_grasp":
            ax.scatter(score, y, s=14, marker="x", color=COLOR[p], linewidth=0.9, zorder=3)
        else:
            ax.scatter(score, y, s=14, facecolor="none", edgecolor=COLOR[p], linewidth=0.9, zorder=3)
ax.axvline(CONF_THRESHOLD, color=INK, linewidth=0.8, linestyle=(0, (3, 2)), zorder=2)
ax.set_yticks(range(len(TYPES)))
ax.set_yticklabels([LABEL[p] for p in TYPES], color=INK)
ax.set_ylim(len(TYPES) - 0.45, -0.55)
ax.set_xlim(-0.03, 1.0)
ax.set_xlabel("Best own-class detection score", color=INK, labelpad=1)
ax.grid(axis="x", color=GRID, linewidth=0.5)
style(ax)


def key(marker, filled=True):
    return plt.Line2D([], [], linestyle="", marker=marker, markersize=4, color=INK,
                      markerfacecolor=INK if filled else "none", markeredgewidth=0.9)


ax.legend([key("o"), key("o", False), key("x"),
           plt.Line2D([], [], color=INK, linewidth=0.8, linestyle=(0, (3, 2)))],
          ["Placed", "Not picked", "Knocked over", f"Threshold, {CONF_THRESHOLD}"],
          frameon=False, fontsize=7, labelcolor=INK, loc="lower center", ncol=4,
          bbox_to_anchor=(0.45, 1.0), handlelength=1.4, columnspacing=1.0)
fig.subplots_adjust(left=0.14, right=0.98, top=0.88, bottom=0.2)
save(fig, "confidence")
