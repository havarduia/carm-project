"""Rebuild data/ and plots/ in this folder from the raw log in raw/.

    python3 make_plots.py

raw/ is a copy of logs/2026-10-11_222859/ (picks.csv, snapshots.csv, the
snapshot images) plus terminal.txt, the terminal output of that run. The
terminal text is what gives the per-move timings and the detections between
picks; neither is in the CSVs.
"""
import csv
import math
import re
import shutil
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
RAW, DATA, PLOTS = HERE / "raw", HERE / "data", HERE / "plots"

BAR, LIGHT, INK, MUTED, GRID = "#3d5a80", "#b9c4d0", "#222222", "#6b6b6b", "#dddddd"
ACCENT = "#c0504d"  # limits and reference lines only

BLOCKED_CURRENT_A = 1.2   # helpers/movement.py at the time of the run
MIN_Z_MM = 6.0            # table limit for the fingertips
GRASP_DEPTH_MM = 15.0     # fingertips stop this far below a part's top
BOX_TO_PART_WIDTH = 0.75  # main/main.py: how the code sizes neighbours

# The ten steps of one pick, in the order the terminal shows them.
PHASES = [
    ("move", "Move above part"),
    ("grip", "Set finger opening"),
    ("move", "Descend onto part"),
    ("grip", "Close gripper"),
    ("move", "Lift part"),
    ("move", "Travel to bin"),
    ("move", "Lower into bin"),
    ("grip", "Release"),
    ("move", "Lift from bin"),
    ("move", "Return home"),
]


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def write(name, rows):
    with open(DATA / name, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


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


# ---------------------------------------------------------------- terminal

STAMP = re.compile(r"^\[INFO\] \[(\d+\.\d+)\] \[move_arm_node\]: (.*)$")


def parse_terminal(lines):
    """Per-pick phase timings and every detection pass, from the terminal text."""
    picks, passes = [], []
    trial, events, events_trial = 0, None, 0
    pending, last_done = None, None

    def close_pick():
        if events is None:
            return
        kinds = [k for k, _, _ in events[:10]]
        assert kinds == [k for k, _ in PHASES], (events_trial, kinds)
        # The trial the pick was started in, not the one being read when it is closed.
        row = {"trial": events_trial,
               "pick_in_trial": sum(p["trial"] == events_trial for p in picks) + 1}
        for (_, label), (_, start, end) in zip(PHASES, events):
            row[label] = round(end - start, 3)
        row["first_move_start"] = events[0][1]
        row["home_done"] = events[9][2]
        picks.append(row)

    for line in lines:
        if line.startswith("===== Trial"):
            trial = int(line.split()[2])
            continue
        if line.startswith("Detected:"):
            classes = Counter()
            if "nothing" not in line:
                for part in line.split(":", 1)[1].split(","):
                    n, cls = part.strip().split("x ", 1)
                    classes[cls] += int(n)
            passes.append({"after": last_done, "classes": classes, "duplicates": 0, "no_depth": 0})
            continue
        if line.startswith("Duplicate detection") and passes:
            passes[-1]["duplicates"] += 1
            continue
        if "valid depth pixels" in line and passes:
            passes[-1]["no_depth"] += 1
            continue
        if line.startswith("Captured target"):
            close_pick()
            events, events_trial = [], trial
            continue
        m = STAMP.match(line)
        if not m:
            continue
        t, text = float(m.group(1)), m.group(2)
        if text.startswith("Cartesian move to"):
            pending = ("move", t)
        elif text.startswith("Setting gripper to"):
            pending = ("grip", t)
        elif text in ("Move complete.", "Gripper done") and pending:
            last_done = t
            if events is not None and len(events) < 10:
                events.append((pending[0], pending[1], t))
            pending = None
    close_pick()
    return picks, passes


# ------------------------------------------------------------------- plots

def plot_phases(timing):
    labels = [label for _, label in PHASES] + ["Detect and plan next pick"]
    means = [sum(r[label] for r in timing) / len(timing) for _, label in PHASES]
    gaps = [r["detect_and_plan_s"] for r in timing if r["detect_and_plan_s"] != ""]
    means.append(sum(gaps) / len(gaps))
    rows = range(len(labels))

    fig, ax = plt.subplots(figsize=(5.2, 3.2), dpi=300)
    colours = [BAR] * len(PHASES) + [LIGHT]
    ax.barh(rows, means, height=0.62, color=colours, zorder=3)
    for row, value in zip(rows, means):
        ax.text(value + 0.05, row, f"{value:.2f} s", va="center", ha="left", color=INK)
    ax.set_yticks(list(rows))
    ax.set_yticklabels(labels, color=INK)
    ax.invert_yaxis()
    ax.set_xlim(0, max(means) * 1.25)
    base(ax, "Where the time of one pick goes (mean of 50 picks)", grid="x")
    ax.spines["bottom"].set_visible(False)
    fig.subplots_adjust(left=0.36, right=0.97, top=0.9, bottom=0.2)
    save(fig, "02_time_per_phase.png",
         f"Mean seconds per step, from the terminal timestamps. The ten arm and gripper steps add up to "
         f"{sum(means[:-1]):.1f} s;\ndetecting again and planning the next grasp adds {means[-1]:.2f} s "
         f"between picks. Only the descent runs at the slow speed.")


def plot_cycle_times(picks):
    seconds = [float(p["seconds"]) for p in picks]
    xs = range(1, len(picks) + 1)
    fig, ax = plt.subplots(figsize=(7.2, 2.4), dpi=300)
    ax.scatter(xs, seconds, s=12, color=BAR, linewidths=0, zorder=3)
    mean = sum(seconds) / len(seconds)
    ax.plot([0.5, len(picks) + 0.5], [mean, mean], color=INK, linewidth=0.8, zorder=2)
    ax.text(len(picks) + 1.2, mean, f"mean {mean:.1f} s", va="center", ha="left", color=INK)
    for edge in range(10, len(picks), 10):
        ax.axvline(edge + 0.5, color=GRID, linewidth=0.8, zorder=1)
    for t in range(len(picks) // 10):
        ax.text(t * 10 + 5.5, max(seconds) + 0.6, f"Trial {t + 1}", ha="center", color=MUTED)
    ax.set_xlim(0, len(picks) + 6)
    ax.set_ylim(0, max(seconds) + 1.6)
    ax.set_xlabel("Pick number", color=MUTED)
    base(ax, "Seconds per pick, in the order they were made")
    fig.subplots_adjust(left=0.06, right=0.98, top=0.87, bottom=0.22)
    save(fig, "03_time_per_pick.png")


def plot_positions(picks):
    fig, ax = plt.subplots(figsize=(4.6, 4.2), dpi=300)
    ys = [float(p["grasp_y"]) for p in picks]
    xs = [float(p["grasp_x"]) for p in picks]
    widths = [float(p["grip_width_mm"]) for p in picks]
    ax.scatter(ys, xs, s=[w * 3.2 for w in widths], color=BAR, alpha=0.6, linewidths=0, zorder=3)
    crowded = [(y, x, w) for y, x, w, p in zip(ys, xs, widths, picks) if p["crowded"] == "1"]
    if crowded:
        ax.scatter([c[0] for c in crowded], [c[1] for c in crowded], s=[c[2] * 3.2 + 60 for c in crowded],
                   facecolors="none", edgecolors=INK, linewidths=0.8, zorder=4)
        ax.annotate("circled: picked with a\nneighbour in the way", (crowded[-1][0], crowded[-1][1]),
                    xytext=(22, -34), textcoords="offset points", color=INK, fontsize=7,
                    arrowprops={"arrowstyle": "-", "color": MUTED, "linewidth": 0.6, "shrinkB": 6})
    ax.set_aspect("equal")
    ax.invert_xaxis()  # +y is to the arm's left, seen from behind the base
    ax.set_xlabel("y in robot base frame (mm), towards the bins  ←", color=MUTED)
    ax.set_ylabel("x in robot base frame (mm), away from the base  →", color=MUTED)
    base(ax, "Where the 50 parts were gripped", grid="both")
    fig.subplots_adjust(left=0.16, right=0.97, top=0.92, bottom=0.2)
    save(fig, "04_pick_positions.png",
         "Each dot is one pick at the position the arm was sent to; dot size follows the measured part width.")


def plot_sizes(picks):
    widths = [float(p["grip_width_mm"]) for p in picks]
    tops = [float(p["top_z"]) for p in picks]
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 2.6), dpi=300)
    a.hist(widths, bins=range(6, 26, 2), color=BAR, rwidth=0.9, zorder=3)
    a.set_xlabel("Gripper opening after closing (mm)", color=MUTED)
    base(a, "Part width, as measured by the gripper")
    b.hist(tops, bins=range(0, 65, 5), color=BAR, rwidth=0.9, zorder=3)
    b.set_xlabel("Height of the top face in the robot base frame (mm)", color=MUTED)
    base(b, "Part height, as read by the camera")
    for ax in (a, b):
        ax.set_ylabel("Parts", color=MUTED)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.3, wspace=0.22)
    save(fig, "05_part_sizes.png",
         "The camera reads heights 2-7 mm low depending on where the part stands (hover checks of 2026-10-11),\n"
         "and the board itself sits 1-2 mm up, so the right panel is not the parts' true height.")


def plot_width_and_opening(picks):
    est = [float(p["width_mm"]) for p in picks]
    grip = [float(p["grip_width_mm"]) for p in picks]
    opening = [float(p["opening_mm"]) for p in picks]
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 3.1), dpi=300)

    top = max(est) + 3
    a.plot([0, top], [0, top], color=MUTED, linewidth=0.7, zorder=2)
    a.plot([0, top], [0, top * BOX_TO_PART_WIDTH], color=ACCENT, linewidth=0.7, zorder=2)
    a.text(top, top, " equal", color=MUTED, va="center", fontsize=7)
    a.text(top, top * BOX_TO_PART_WIDTH, f" {BOX_TO_PART_WIDTH} × box", color=ACCENT, va="center", fontsize=7)
    a.scatter(est, grip, s=12, color=BAR, alpha=0.7, linewidths=0, zorder=3)
    a.set_xlim(0, top + 7)
    a.set_ylim(0, top)
    a.set_xlabel("Width estimated from the detection box (mm)", color=MUTED)
    a.set_ylabel("Width measured by the gripper (mm)", color=MUTED)
    base(a, "How good is the width estimate?", grid="both")

    top = 44
    b.plot([0, top], [0, top], color=MUTED, linewidth=0.7, zorder=2)
    b.text(top, top, " no clearance", color=MUTED, va="center", fontsize=7)
    b.scatter(grip, opening, s=12, color=BAR, alpha=0.7, linewidths=0, zorder=3)
    b.set_xlim(0, top + 9)
    b.set_ylim(0, top)
    b.set_xlabel("Width measured by the gripper (mm)", color=MUTED)
    b.set_ylabel("Finger opening on the way down (mm)", color=MUTED)
    base(b, "How far open were the fingers?", grid="both")

    ratio = sorted(g / e for g, e in zip(grip, est))
    spare = sorted((o - g) / 2 for o, g in zip(opening, grip))
    fig.subplots_adjust(left=0.08, right=0.97, top=0.9, bottom=0.27, wspace=0.28)
    save(fig, "06_width_estimate_and_opening.png",
         f"Left: the box reads wider than the part; measured / estimated has median {ratio[len(ratio) // 2]:.2f}. "
         f"Right: free space per side between finger and part,\nmedian {spare[len(spare) // 2]:.1f} mm, "
         f"smallest {spare[0]:.1f} mm. The gripper opens to 40 mm at most.")


def plot_yaw(picks):
    counts = Counter(int(p["yaw_deg"]) for p in picks)
    angles = list(range(-75, 91, 15))
    fig, ax = plt.subplots(figsize=(5.2, 2.5), dpi=300)
    ax.bar(range(len(angles)), [counts[a] for a in angles], width=0.7, color=BAR, zorder=3)
    for i, a in enumerate(angles):
        if counts[a]:
            ax.text(i, counts[a] + 0.2, str(counts[a]), ha="center", va="bottom", color=INK)
    ax.set_xticks(range(len(angles)))
    ax.set_xticklabels([f"{a}°" for a in angles])
    ax.set_yticks(range(0, max(counts.values()) + 2, 5))
    ax.set_ylabel("Picks", color=MUTED)
    ax.set_xlabel("Gripper rotation about the vertical", color=MUTED)
    base(ax, "Gripper angle chosen for each pick")
    fig.subplots_adjust(left=0.1, right=0.98, top=0.88, bottom=0.3)
    save(fig, "07_gripper_angle.png",
         "At 0° the fingers close along the base y axis. The angle is the one whose finger landing strips are freest.")


def plot_current(picks):
    current = [float(p["descent_current_a"]) for p in picks]
    xs = range(1, len(picks) + 1)
    fig, ax = plt.subplots(figsize=(7.2, 2.4), dpi=300)
    ax.scatter(xs, current, s=12, color=BAR, linewidths=0, zorder=3)
    ax.axhline(BLOCKED_CURRENT_A, color=ACCENT, linewidth=0.9, zorder=2)
    ax.text(len(picks) + 6.6, BLOCKED_CURRENT_A + 0.04, f"descent is stopped above {BLOCKED_CURRENT_A} A",
            va="bottom", ha="right", color=ACCENT)
    for edge in range(10, len(picks), 10):
        ax.axvline(edge + 0.5, color=GRID, linewidth=0.8, zorder=1)
    ax.set_xlim(0, len(picks) + 7)
    ax.set_ylim(0, BLOCKED_CURRENT_A * 1.25)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_xlabel("Pick number", color=MUTED)
    ax.set_ylabel("A", color=MUTED)
    base(ax, "Largest jump in shoulder or elbow motor current during each descent")
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.3)
    worst = max(current)
    save(fig, "08_descent_current.png",
         f"A descent is stopped as blocked if the current jumps by more than {BLOCKED_CURRENT_A} A within 0.2 s. "
         f"None was; the largest free descent reached {worst:.2f} A\n(pick {current.index(worst) + 1}, "
         f"the tallest part of trial 3). Pressing on the gripper by hand gave 2-5 A.")


def plot_detections(det):
    trials = sorted({d["trial"] for d in det})
    fig, axes = plt.subplots(1, len(trials), sharey=True, figsize=(7.2, 2.5), dpi=300)
    for ax, trial in zip(axes, trials):
        rows = [d for d in det if d["trial"] == trial]
        xs = [d["picks_done"] for d in rows]
        ax.plot(xs, [d["parts_on_board"] for d in rows], color=MUTED, linewidth=0.9, zorder=2,
                label="parts on the board")
        ax.scatter(xs, [d["detections"] for d in rows], s=11, color=BAR, linewidths=0, zorder=3,
                   label="detections")
        ax.set_xticks([0, 5, 10])
        ax.set_yticks([0, 5, 10])
        base(ax, f"Trial {trial}")
    axes[0].set_ylabel("Count", color=MUTED)
    axes[len(trials) // 2].set_xlabel("Picks done in the trial", color=MUTED)
    axes[0].legend(loc="upper right", frameon=False, fontsize=6.5, handlelength=1.4, borderaxespad=0.0,
                   labelcolor=MUTED)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.88, bottom=0.34, wspace=0.1)
    extra = sum(d["detections"] - d["parts_on_board"] for d in det if d["detections"] > d["parts_on_board"])
    wrong = sum(d["transformer"] + d["resistor"] for d in det)
    save(fig, "09_detections_between_picks.png",
         f"The board is detected again after every pick ({len(det)} passes). Dots above the line are extra boxes: "
         f"a second box on one part, or a part already in the bin\n(the bin is in view). Summed over all passes "
         f"there are {extra} extra boxes, {wrong} of them labelled transformer or resistor. "
         f"No pass has fewer detections than parts.")


def plot_grasp_height(picks):
    tops = [float(p["top_z"]) for p in picks]
    grasp = [float(p["grasp_z"]) for p in picks]
    fig, ax = plt.subplots(figsize=(4.6, 3.2), dpi=300)
    top = max(tops) + 4
    ax.plot([0, top], [-GRASP_DEPTH_MM, top - GRASP_DEPTH_MM], color=MUTED, linewidth=0.7, zorder=2)
    ax.axhline(MIN_Z_MM, color=ACCENT, linewidth=0.8, zorder=2)
    ax.text(top, MIN_Z_MM + 1.2, "table limit 6 mm", color=ACCENT, ha="right", fontsize=7)
    ax.text(top * 0.72, top * 0.72 - GRASP_DEPTH_MM - 6, "15 mm below the top", color=MUTED, fontsize=7, rotation=33)
    ax.scatter(tops, grasp, s=12, color=BAR, alpha=0.7, linewidths=0, zorder=3)
    ax.set_xlim(0, top)
    ax.set_ylim(0, top - GRASP_DEPTH_MM + 4)
    ax.set_xlabel("Height of the part's top face (mm)", color=MUTED)
    ax.set_ylabel("Height the fingertips stopped at (mm)", color=MUTED)
    base(ax, "How low the fingers went", grid="both")
    fig.subplots_adjust(left=0.14, right=0.97, top=0.9, bottom=0.26)
    clamped = sum(g <= MIN_Z_MM for g in grasp)
    save(fig, "10_grasp_height.png",
         f"The fingertips aim 15 mm below a part's top and never go below the table limit.\n"
         f"{clamped} of {len(picks)} picks were at that limit: the part was too short for the full depth.")


def plot_trial_durations(summary):
    rows = range(len(summary))
    fig, ax = plt.subplots(figsize=(5.2, 2.3), dpi=300)
    values = [r["duration_s"] for r in summary]
    ax.barh(rows, values, height=0.62, color=BAR, zorder=3)
    for row, r in zip(rows, summary):
        ax.text(r["duration_s"] + 2, row, f"{r['duration_s']:.0f} s  ({r['parts']} parts)", va="center", color=INK)
    ax.set_yticks(list(rows))
    ax.set_yticklabels([f"Trial {r['trial']}" for r in summary], color=INK)
    ax.invert_yaxis()
    ax.set_xlim(0, max(values) * 1.35)
    base(ax, "Time to clear the board", grid="x")
    ax.spines["bottom"].set_visible(False)
    fig.subplots_adjust(left=0.13, right=0.97, top=0.88, bottom=0.26)
    save(fig, "11_time_to_clear_board.png",
         "From the first arm move of the trial to the arm back home after the last part, including every re-detection.")


def main():
    DATA.mkdir(exist_ok=True)
    PLOTS.mkdir(exist_ok=True)
    plt.rcParams.update({"font.family": ["Nimbus Sans", "Liberation Sans", "DejaVu Sans"], "font.size": 8})

    picks = read(RAW / "picks.csv")
    trials = read(RAW / "snapshots.csv")
    timing, passes = parse_terminal((RAW / "terminal.txt").read_text().splitlines())
    assert len(timing) == len(picks), (len(timing), len(picks))

    # Time between the arm getting home and setting off again: detection plus planning.
    for i, row in enumerate(timing):
        same_trial = i > 0 and timing[i - 1]["trial"] == row["trial"]
        row["detect_and_plan_s"] = round(row["first_move_start"] - timing[i - 1]["home_done"], 3) if same_trial else ""
        row["arm_and_gripper_s"] = round(sum(row[label] for _, label in PHASES), 3)
    write("pick_phase_times.csv", [
        {k: v for k, v in row.items() if k not in ("first_move_start", "home_done")} for row in timing])

    # One row per detection pass. Each trial has one before its first pick and one after every pick.
    det, index = [], 0
    for t in trials:
        n = int(t["located"])
        for k in range(n + 1):
            p = passes[index]
            index += 1
            det.append({
                "trial": int(t["snapshot"]), "picks_done": k, "parts_on_board": n - k,
                "detections": sum(p["classes"].values()),
                "capacitor": p["classes"]["capacitor"], "transformer": p["classes"]["transformer"],
                "resistor": p["classes"]["resistor"],
                "dropped_as_duplicate": p["duplicates"], "dropped_no_depth": p["no_depth"],
            })
    assert index == len(passes), (index, len(passes))
    write("detection_passes.csv", det)

    summary = []
    for t in trials:
        rows = [r for r in timing if r["trial"] == int(t["snapshot"])]
        mine = [p for p in picks if p["snapshot"] == t["snapshot"]]
        summary.append({
            "trial": int(t["snapshot"]), "parts": int(t["located"]), "placed": int(t["placed"]),
            "successes": int(t["successes"]), "detections_first_image": int(t["detected"]),
            "duration_s": round(rows[-1]["home_done"] - rows[0]["first_move_start"], 1),
            "mean_s_per_pick": round(sum(float(p["seconds"]) for p in mine) / len(mine), 2),
            "max_descent_current_a": max(float(p["descent_current_a"]) for p in mine),
            "crowded_picks": sum(p["crowded"] == "1" for p in mine),
            "grasps_at_table_limit": sum(float(p["grasp_z"]) <= MIN_Z_MM for p in mine),
        })
    write("trial_summary.csv", summary)

    shutil.copy(RAW / "kpi_trials.png", PLOTS / "01_kpi_trials.png")
    print("saved plots/01_kpi_trials.png")
    plot_phases(timing)
    plot_cycle_times(picks)
    plot_positions(picks)
    plot_sizes(picks)
    plot_width_and_opening(picks)
    plot_yaw(picks)
    plot_current(picks)
    plot_detections(det)
    plot_grasp_height(picks)
    plot_trial_durations(summary)


if __name__ == "__main__":
    main()
