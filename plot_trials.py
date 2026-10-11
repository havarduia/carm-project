"""Plot the KPIs of one run of main.py: per trial, how many parts were picked
successfully, how long each pick took, and how many detections the model made.

Reads logs/<RUN>/snapshots.csv and picks.csv and saves logs/<RUN>/kpi_trials.png.

    python3 plot_trials.py                       # the newest run under logs/
    python3 plot_trials.py --run 2026-10-11_222859
"""
import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOGS = Path(__file__).resolve().parent / "logs"

BAR, LIGHT, INK, MUTED, GRID = "#3d5a80", "#b9c4d0", "#222222", "#6b6b6b", "#dddddd"


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def number(row, key):
    return int(row[key] or 0)


def style(ax, title):
    ax.set_title(title, loc="left", fontsize=8.5, color=INK, pad=6)
    ax.tick_params(axis="x", length=0, pad=3, colors=MUTED)
    ax.tick_params(axis="y", length=0, pad=6)
    ax.grid(axis="x", color=GRID, linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "bottom"):
        ax.spines[side].set_visible(False)
    ax.spines["left"].set_color(INK)
    ax.spines["left"].set_linewidth(0.6)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run", help="folder name under logs/ (default: the newest with a snapshots.csv)")
    args = parser.parse_args()
    if args.run:
        run = LOGS / args.run
    else:
        run = max(p.parent for p in LOGS.glob("*/snapshots.csv"))

    trials = read(run / "snapshots.csv")
    picks = read(run / "picks.csv")
    rows = range(len(trials))
    names = [f"Trial {t['snapshot']}" for t in trials]

    located = [number(t, "located") for t in trials]
    # Without the operator's answers a placed part is the best that is known.
    success = [number(t, "successes") if t["successes"] else number(t, "placed") for t in trials]
    detected = [number(t, "detected") for t in trials]
    seconds = [[float(p["seconds"]) for p in picks
                if p["snapshot"] == t["snapshot"] and p["outcome"] == "placed"] for t in trials]

    plt.rcParams.update({"font.family": ["Nimbus Sans", "Liberation Sans", "DejaVu Sans"], "font.size": 8})
    fig, (ax_ok, ax_time, ax_det) = plt.subplots(1, 3, sharey=True, figsize=(7.2, 2.7), dpi=300)

    # Parts picked successfully, against the parts that were on the board.
    ax_ok.barh(rows, located, height=0.62, color=LIGHT, zorder=2)
    ax_ok.barh(rows, success, height=0.62, color=BAR, zorder=3)
    for row, (ok, n) in enumerate(zip(success, located)):
        ax_ok.text(n + 0.3, row, f"{ok} / {n}", va="center", ha="left", color=INK)
    ax_ok.set_xlim(0, max(located) * 1.3)
    style(ax_ok, "Parts picked and binned")

    # One dot per pick; the tick is the trial's mean.
    for row, values in enumerate(seconds):
        if not values:
            continue
        ax_time.scatter(values, [row] * len(values), s=7, color=BAR, alpha=0.55, linewidths=0, zorder=3)
        mean = sum(values) / len(values)
        ax_time.plot([mean, mean], [row - 0.3, row + 0.3], color=INK, linewidth=1.0, zorder=4)
        ax_time.text(max(values) + 0.5, row, f"{mean:.1f} s", va="center", ha="left", color=INK)
    longest = max((max(v) for v in seconds if v), default=1.0)
    ax_time.set_xlim(0, longest * 1.3)
    style(ax_time, "Time per pick (s)")

    # Detections in the first image of the trial; the excess over the parts
    # that got a position is double boxes and detections outside the board.
    ax_det.barh(rows, detected, height=0.62, color=LIGHT, zorder=2)
    ax_det.barh(rows, located, height=0.62, color=BAR, zorder=3)
    for row, (d, n) in enumerate(zip(detected, located)):
        ax_det.text(d + 0.3, row, f"{d}" if d == n else f"{d}  (+{d - n})", va="center", ha="left", color=INK)
    ax_det.set_xlim(0, max(detected) * 1.35)
    style(ax_det, "Model detections")

    ax_ok.set_yticks(list(rows))
    ax_ok.set_yticklabels(names, color=INK)
    ax_ok.invert_yaxis()

    all_seconds = [s for values in seconds for s in values]
    total = lambda key: sum(number(t, key) for t in trials)
    mean_s = f"{sum(all_seconds) / len(all_seconds):.1f} s" if all_seconds else "n/a"
    footer = (
        f"{len(trials)} trials, {sum(located)} parts: {sum(success)} picked and binned "
        f"({100 * sum(success) / max(sum(located), 1):.0f} %), mean {mean_s} per pick. "
        f"Missed by camera {total('missed_by_camera')}, empty grasps {total('empty_grasp')}, "
        f"blocked descents {total('blocked_descent')}, left on board {total('remaining')}, "
        f"neighbours knocked {total('knocked_over')}.\n"
        f"Detections beyond the parts on the board (+) are double boxes on one part or lie outside "
        f"the pick area; they are filtered out before picking.   Run {run.name}."
    )
    fig.text(0.008, 0.02, footer, fontsize=6.5, color=MUTED, ha="left", va="bottom", linespacing=1.5)
    fig.subplots_adjust(left=0.085, right=0.985, top=0.89, bottom=0.25, wspace=0.14)

    out = run / "kpi_trials.png"
    fig.savefig(out, facecolor="white")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
