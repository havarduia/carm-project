"""The KPIs of the five trials of 2026-10-26 as one IEEE double-column figure
(7.16 in wide), with no title or caption text: per trial, the parts placed in
the right bin and the parts detected in the first image, each out of the ten
on the board, and the time of each placed pick (tick: the trial's mean).

Writes kpi.pdf (vector, for LaTeX) and kpi.png.
"""
import csv
import os

import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
BAR, TRACK, INK, GRID = "#2a78d6", "#dcdbd6", "#2b2b2a", "#e6e5e1"

trials = list(csv.DictReader(open(os.path.join(HERE, "raw", "snapshots.csv"))))
picks = list(csv.DictReader(open(os.path.join(HERE, "raw", "picks.csv"))))
on_board = [sum(int(t[k]) for k in t if k.startswith("on_board_")) for t in trials]
placed = [int(t["placed"]) for t in trials]
detected = [int(t["detected"]) for t in trials]
seconds = [[float(p["seconds"]) for p in picks
            if p["snapshot"] == t["snapshot"] and p["outcome"] == "placed"] for t in trials]
rows = range(len(trials))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Nimbus Roman", "Times New Roman", "DejaVu Serif"],
                     "font.size": 7, "pdf.fonttype": 42})
fig, axes = plt.subplots(1, 3, sharey=True, figsize=(7.16, 1.6))


def bars(ax, values, xlabel):
    ax.barh(rows, on_board, height=0.6, color=TRACK, zorder=2)
    ax.barh(rows, values, height=0.6, color=BAR, zorder=3)
    for row, (v, n) in enumerate(zip(values, on_board)):
        ax.text(n + 0.25, row, f"{v}/{n}", va="center", ha="left", color=INK)
    ax.set_xlim(0, max(on_board) * 1.18)
    ax.set_xticks(range(0, max(on_board) + 1, 2))
    ax.set_xlabel(xlabel, color=INK, labelpad=1)


bars(axes[0], placed, "Parts placed in the right bin")
bars(axes[1], detected, "Parts detected in the first image")

ax = axes[2]
for row, values in enumerate(seconds):
    ax.scatter(values, [row] * len(values), s=7, color=BAR, alpha=0.6, linewidths=0, zorder=3)
    mean = sum(values) / len(values)
    ax.plot([mean, mean], [row - 0.3, row + 0.3], color=INK, linewidth=0.8, zorder=4)
    ax.text(max(values) + 0.12, row, f"{mean:.1f} s", va="center", ha="left", color=INK)
ax.set_xlim(10, 13.5)
ax.set_xticks([10, 11, 12, 13])
ax.grid(axis="x", color=GRID, linewidth=0.5)
ax.set_xlabel("Time per pick (s)", color=INK, labelpad=1)

for ax in axes:
    ax.spines[["top", "right"]].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK)
        ax.spines[side].set_linewidth(0.5)
    ax.tick_params(colors=INK, labelsize=7, length=2, width=0.5)
axes[0].set_yticks(list(rows))
axes[0].set_yticklabels([f"Trial {t['snapshot']}" for t in trials])
axes[0].invert_yaxis()
fig.subplots_adjust(left=0.07, right=0.985, top=0.97, bottom=0.22, wspace=0.12)

for ext in ("pdf", "png"):
    fig.savefig(os.path.join(HERE, f"kpi.{ext}"), dpi=300, facecolor="white")
