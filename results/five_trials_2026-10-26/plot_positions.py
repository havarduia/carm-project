"""Where every part of the five trials of 2026-10-26 was, all trials in one
IEEE single-column plot (3.5 in wide), in the robot's frame: the picks from
raw/picks.csv, and the parts never picked from unpicked_positions.csv (read off
the trial's first camera frame, ray to the board surface).

Writes part_positions.pdf (vector, for LaTeX) and part_positions.png.
"""
import csv
import os

import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OFFSET_X, OFFSET_Y = -14, 10  # main.py: grasp point = top-face centre + offset

# Reference categorical palette, slots 1-3 in order; marker shape repeats the class.
COLOR = {"capacitor": "#2a78d6", "coil": "#eb6834", "resistor": "#1baf7a"}
SHAPE = {"capacitor": "o", "coil": "s", "resistor": "^"}
INK, MUTED, GRID = "#2b2b2a", "#6f6e6a", "#e6e5e1"
SIZE = 18

picks = list(csv.DictReader(open(os.path.join(HERE, "raw", "picks.csv"))))
unpicked = list(csv.DictReader(open(os.path.join(HERE, "unpicked_positions.csv"))))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Nimbus Roman", "Times New Roman", "DejaVu Serif"],
                     "font.size": 8, "pdf.fonttype": 42})
fig, ax = plt.subplots(figsize=(3.5, 2.85))

# Every marker as (x, y, part, what); what is placed, empty or missed.
points = []
for r in picks:
    part = "coil" if r["class"] == "transformer" else r["class"]
    points.append((float(r["grasp_x"]) - OFFSET_X, float(r["grasp_y"]) - OFFSET_Y, part,
                   "placed" if r["outcome"] == "placed" else "empty"))
for r in unpicked:
    points.append((float(r["x_mm"]), float(r["y_mm"]), r["part"], "missed"))

# Markers of one kind closer than this are one marker, labelled with the count:
# parts were set out in nearly the same spots from trial to trial.
SAME_SPOT_MM = 5.0
groups = []
for x, y, part, what in points:
    for g in groups:
        if g[2:4] == [part, what] and (g[0] - x) ** 2 + (g[1] - y) ** 2 < SAME_SPOT_MM ** 2:
            g[4] += 1
            break
    else:
        groups.append([x, y, part, what, 1])

for x, y, part, what, n in groups:
    if what == "placed":
        ax.scatter(x, y, s=SIZE, marker=SHAPE[part], color=COLOR[part],
                   edgecolor="white", linewidth=0.6, zorder=3)
    elif what == "empty":
        ax.scatter(x, y, s=SIZE, marker="x", color=COLOR[part], linewidth=1, zorder=3)
    else:
        ax.scatter(x, y, s=SIZE, marker=SHAPE[part], facecolor="none",
                   edgecolor=COLOR[part], linewidth=0.9, zorder=4)
    if n > 1:
        # Up and left of the marker on a leader line, or down and left when
        # another marker is in the way.
        up = not any(-25 < px - x < 5 and SAME_SPOT_MM < py - y < 22 for px, py, *_ in points)
        ax.annotate(f"{n}×", (x, y), xytext=(-9, 9 if up else -9), textcoords="offset points",
                    ha="right", va="center", fontsize=6.5, color=INK, zorder=5,
                    arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.5,
                                    shrinkA=1, shrinkB=3.5))

ax.set_aspect("equal")
ax.set_xlim(300, 590)
ax.set_ylim(-100, 110)
ax.grid(color=GRID, linewidth=0.5)
ax.tick_params(colors=INK, labelsize=7, length=2, width=0.5)
ax.spines[["top", "right"]].set_visible(False)
for side in ("left", "bottom"):
    ax.spines[side].set_color(INK)
    ax.spines[side].set_linewidth(0.5)
ax.set_xlabel("x (mm)", color=INK, fontsize=7, labelpad=1)
ax.set_ylabel("y (mm)", color=INK, fontsize=7, labelpad=1)


def key(marker, color, filled=True):
    return plt.Line2D([], [], linestyle="", marker=marker, markersize=4.5,
                      markerfacecolor=color if filled else "none", markeredgecolor=color,
                      markeredgewidth=0.6 if filled else 0.9)


fig.legend(
    [key("o", COLOR["capacitor"]), key("o", COLOR["capacitor"], False),
     key("s", COLOR["coil"]), key("s", COLOR["coil"], False),
     key("x", COLOR["capacitor"]), key("^", COLOR["resistor"], False)],
    ["Capacitor placed", "Capacitor not detected", "Coil placed", "Coil not detected",
     "Empty grasp", "Resistor not detected"],
    loc="lower center", ncol=3, bbox_to_anchor=(0.53, 0.0), frameon=False, fontsize=7,
    labelcolor=INK, handletextpad=0.2, columnspacing=1.0)
fig.subplots_adjust(left=0.14, right=0.98, top=0.98, bottom=0.21)

for ext in ("pdf", "png"):
    fig.savefig(os.path.join(HERE, f"part_positions.{ext}"), dpi=300, facecolor="white")
