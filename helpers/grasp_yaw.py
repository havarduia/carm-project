"""Choose the gripper rotation whose open fingers come down on free space.

The gripper always points straight down; the only freedom is the rotation
about the vertical axis (yaw). For each candidate yaw the two strips of table
the open fingers descend through are checked against the snapshot's depth
points, so anything standing there blocks that yaw - whether or not the
detector reported it.
"""

import math

import numpy as np

# xArm gripper fingers, in mm. They close along the tool Y axis (link_tcp ->
# left_finger / right_finger sit at y = +/-31 mm). Thickness is along the
# closing direction, width across it. Estimates - measure the fingers and
# correct these if the check proves too tight or too loose.
FINGER_THICKNESS_MM = 12.0
FINGER_WIDTH_MM = 25.0

# Kept clear around each finger, to absorb calibration and depth error.
MARGIN_MM = 4.0

# Depth points this far around the grasp are used: far enough to cover both
# finger strips at any yaw, near enough that the floor found is the local one.
SEARCH_RADIUS_MM = 50.0

# The camera's height reading drifts by up to ~1 cm across the table, so
# heights are measured from the floor it sees nearby, not from its absolute z.
# That floor is the table or the PCB lying on it, whichever fills the
# neighbourhood; the PCB reads about 6 mm above the table.
FLOOR_PERCENTILE = 25

# A point is an obstacle if it stands higher above that floor than the
# fingertips are above the table. Because the floor may be the PCB, parts
# shorter than about 10 mm are not seen as obstacles.
#
# This many obstacle points in a strip block it; fewer is depth noise. A
# 5 mm part is ~70 points at the working distance.
MIN_OBSTACLE_POINTS = 40

# The camera looks from one side, so it has no depth behind tall parts. Strips
# are split into cells this size, and a strip with more than this fraction of
# cells unseen is treated as blocked.
CELL_MM = 4.0
MAX_UNSEEN_FRACTION = 0.35

# Candidate yaws in degrees, smallest rotation first. The fingers are
# symmetric, so 180 degrees covers every distinct orientation.
YAWS_DEG = (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90)


def closing_axis(yaw):
    """Unit vector (x, y) in link_base that the fingers close along at `yaw`."""
    return -math.sin(yaw), math.cos(yaw)


def local_scene(cloud_mm, x, y):
    """Depth points around a grasp at (x, y): offsets (N, 2) and heights (N,)
    above the local floor, in mm."""
    d = cloud_mm[:, :2] - (x, y)
    near = np.hypot(d[:, 0], d[:, 1]) <= SEARCH_RADIUS_MM
    d, z = d[near], cloud_mm[near, 2]
    if len(z) == 0:
        return d, z
    return d, z - np.percentile(z, FLOOR_PERCENTILE)


def strip_report(d, height, tip_height, gap_mm, yaw):
    """(blocked, crowding) for the two finger strips at one yaw.

    `d` and `height` come from local_scene(); `tip_height` is how high the
    fingertips stop above the table. `crowding` counts obstacle points in a
    slightly wider strip, to rank the yaws that are free.
    """
    if len(height) == 0:
        return True, 0

    tall = height > tip_height

    cx, cy = closing_axis(yaw)
    along = d[:, 0] * cx + d[:, 1] * cy      # closing direction
    across = d[:, 0] * cy - d[:, 1] * cx

    inner = gap_mm / 2.0 - MARGIN_MM
    depth = FINGER_THICKNESS_MM + 2 * MARGIN_MM
    half_width = FINGER_WIDTH_MM / 2.0 + MARGIN_MM

    def in_strip(extra):
        return ((np.abs(across) <= half_width + extra)
                & (np.abs(along) >= inner - extra)
                & (np.abs(along) <= inner + depth + extra))

    strip = in_strip(0.0)
    blocked = int(tall[strip].sum()) >= MIN_OBSTACLE_POINTS

    n_across = math.ceil(2 * half_width / CELL_MM)
    n_along = math.ceil(depth / CELL_MM)
    for side in (along > 0, along < 0):
        s = strip & side
        ia = np.clip(((across[s] + half_width) / CELL_MM).astype(int), 0, n_across - 1)
        il = np.clip(((np.abs(along[s]) - inner) / CELL_MM).astype(int), 0, n_along - 1)
        seen = len(np.unique(ia * n_along + il))
        if 1.0 - seen / (n_across * n_along) > MAX_UNSEEN_FRACTION:
            blocked = True

    return blocked, int(tall[in_strip(6.0)].sum())


def choose_grasp_yaw(cloud_mm, x, y, grasp_z, gap_mm, floor_z):
    """Yaw in radians for a grasp at (x, y, grasp_z), or None if every yaw is blocked.

    `cloud_mm` is the snapshot's depth points, (N, 3) mm in link_base.
    `gap_mm` is the opening between the fingers on the way down and `floor_z`
    the real height of the table in link_base.
    """
    d, height = local_scene(cloud_mm, x, y)
    best = None
    for deg in YAWS_DEG:
        yaw = math.radians(deg)
        blocked, crowding = strip_report(d, height, grasp_z - floor_z, gap_mm, yaw)
        if not blocked and (best is None or crowding < best[0]):
            best = (crowding, yaw)
    return None if best is None else best[1]
