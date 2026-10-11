"""Find the top face of a detected part in the snapshot's depth points.

The detector's own point is the depth at the centre of the detection box.
The camera looks down at an angle, so on a standing part that pixel lies on
the side wall facing the camera: too low, and about one part radius short of
the part's axis. The top face is in the depth points all the same, and its
centre and height are what the gripper needs.
"""

import math

import numpy as np

# Points within this of the highest ones count as the top face. Wide enough
# for depth noise and a domed capacitor top, small against a part's height.
TOP_BAND_MM = 4.0

# The top is this percentile of the heights over the part, not the maximum,
# so a few stray depth pixels do not set it.
TOP_PERCENTILE = 90

# Fewer points than this over the part and the depth there is not trusted.
MIN_POINTS = 30

# The centre is found, then looked for again around where it was found.
ROUNDS = 2


def part_top(cloud_mm, seed, radius_mm, camera_xy):
    """(x, y, z) of the centre of a part's top face, in mm, or None.

    `cloud_mm` is (N, 3) depth points in link_base, `seed` the detector's
    (x, y, z) point on the part, `radius_mm` half the part's width and
    `camera_xy` where the camera stands, to know which way is "behind" the
    seed.
    """
    away = np.subtract(seed[:2], camera_xy)
    distance = math.hypot(*away)
    if len(cloud_mm) == 0 or distance == 0:
        return None

    # First guess: the axis is one radius behind the point the camera hit.
    centre = np.asarray(seed[:2], dtype=float) + away / distance * radius_mm

    # Only what is near the part matters, and it is searched more than once.
    near = cloud_mm[np.hypot(*(cloud_mm[:, :2] - centre).T) <= 3 * radius_mm]

    top_z = None
    for _ in range(ROUNDS):
        over = near[np.hypot(*(near[:, :2] - centre).T) <= radius_mm]
        if len(over) < MIN_POINTS:
            return None
        top_z = float(np.percentile(over[:, 2], TOP_PERCENTILE))
        centre = over[over[:, 2] >= top_z - TOP_BAND_MM, :2].mean(axis=0)
    return float(centre[0]), float(centre[1]), top_z
