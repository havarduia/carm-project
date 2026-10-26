"""Two IEEE single-column figures (3.5 in wide) from the picks of the five
trials of 2026-10-26, with no title or caption text:

grip_width:      the width the gripper closed to on each placed part, by type.
                 The two empty grasps (closed to under 1 mm) measured nothing
                 and are left out.
descent_current: the largest change in shoulder or elbow motor current within
                 0.2 s during each descent, with the normal distribution fitted
                 to it and the 1.2 A at which a descent is stopped.

Writes grip_width.pdf/.png and descent_current.pdf/.png.
"""
import csv
import math
import os
import statistics

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BLOCKED_CURRENT_A = 1.2  # helpers/movement.py
COLOR = {"capacitor": "#2a78d6", "coil": "#eb6834"}
INK, GRID = "#2b2b2a", "#e6e5e1"

picks = list(csv.DictReader(open(os.path.join(HERE, "raw", "picks.csv"))))
for r in picks:
    r["part"] = "coil" if r["class"] == "transformer" else r["class"]

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Nimbus Roman", "Times New Roman", "DejaVu Serif"],
                     "font.size": 7, "pdf.fonttype": 42})


def style(ax, xlabel):
    ax.spines[["top", "right"]].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK)
        ax.spines[side].set_linewidth(0.5)
    ax.tick_params(colors=INK, labelsize=7, length=2, width=0.5)
    ax.grid(axis="y", color=GRID, linewidth=0.5)
    ax.set_axisbelow(True)
    ax.set_xlabel(xlabel, color=INK, labelpad=1)
    ax.set_ylabel("Picks", color=INK, labelpad=2)
    ax.legend(frameon=False, fontsize=7, labelcolor=INK, handlelength=1.2, loc="upper left")


def save(fig, name):
    fig.subplots_adjust(left=0.11, right=0.98, top=0.97, bottom=0.2)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(HERE, f"{name}.{ext}"), dpi=300, facecolor="white")


# Width the gripper closed to, stacked by type in 1 mm bins.
fig, ax = plt.subplots(figsize=(3.5, 1.9))
widths = {p: [float(r["grip_width_mm"]) for r in picks if r["outcome"] == "placed" and r["part"] == p]
          for p in COLOR}
ax.hist(list(widths.values()), bins=np.arange(6, 27, 1.0), stacked=True,
        color=list(COLOR.values()), edgecolor="white", linewidth=0.5,
        label=[f"Capacitor (n = {len(widths['capacitor'])})", f"Coil (n = {len(widths['coil'])})"])
ax.set_xlim(6, 26)
ax.set_xticks(range(6, 27, 2))
ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
style(ax, "Part width measured by the gripper (mm)")
save(fig, "grip_width")

# Largest current change of every descent, with a fitted normal distribution.
fig, ax = plt.subplots(figsize=(3.5, 1.9))
current = [float(r["descent_current_a"]) for r in picks if r["descent_current_a"]]
mu, sigma = statistics.mean(current), statistics.stdev(current)
step = 0.05
ax.hist(current, bins=np.arange(0.2, 1.3 + step, step), color=COLOR["capacitor"],
        edgecolor="white", linewidth=0.5, label=f"Descents (n = {len(current)})")
x = np.linspace(0, 1.4, 300)
ax.plot(x, len(current) * step / (sigma * math.sqrt(2 * math.pi)) * np.exp(-0.5 * ((x - mu) / sigma) ** 2),
        color=INK, linewidth=0.9, label=f"Normal fit, μ = {mu:.2f} A, σ = {sigma:.2f} A")
ax.axvline(BLOCKED_CURRENT_A, color=INK, linewidth=0.9, linestyle=(0, (3, 2)),
           label=f"Stop threshold, {BLOCKED_CURRENT_A} A")
ax.set_xlim(0, 1.4)
ax.set_ylim(0, 10.5)  # room for the legend above the bars
ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
style(ax, "Largest shoulder or elbow current change in 0.2 s (A)")
save(fig, "descent_current")
