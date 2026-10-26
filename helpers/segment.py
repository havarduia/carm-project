"""Cut one detected part out of the depth image, apart from its neighbours.

The detection box holds the part, but in a packed group also the edges of
the parts around it. Inside the box, the part is what stands above the board
and is reached from the middle of the box without crossing a jump in depth:
where one part stands against another, the camera sees the front one's edge
against the other's wall further back, so the two never join. The part's own
points give its top face, which helpers/part_top.py, searching around the
detection, could take from a taller neighbour.

Not its width: on the glossy tops and walls of the capacitors the depth has
holes and smears, and widths read from it were off by -50% to +35% against
what the gripper closed to (2026-10-26). The detection box stays the width.
"""

import cv2
import numpy as np

# Neighbouring pixels further apart than this in 3D lie on different
# surfaces. On one surface they are ~0.5 mm apart at the working distance.
EDGE_MM = 3.0

# Points this far above the floor are part of something standing on it.
MIN_HEIGHT_MM = 4.0

# The floor is this percentile of the heights in the box and a margin of
# this many box sizes around it, where the board shows between the parts.
FLOOR_PERCENTILE = 5
FLOOR_MARGIN = 0.5

# As in part_top: the top is this percentile of the part's heights, and
# points within TOP_BAND_MM of it are the top face.
TOP_PERCENTILE = 90
TOP_BAND_MM = 4.0

# Fewer points than this on the part or its top face, and it is not trusted.
MIN_POINTS = 30


def segment_top(grid_mm, box):
    """(x, y, z) of the centre of the part's top face, in mm, or None if the
    part cannot be cut out.

    `grid_mm` is the depth image as (H, W, 3) points in link_base, NaN where
    there is no reading, and `box` the detection as (x1, y1, x2, y2) pixels.
    """
    h, w = grid_mm.shape[:2]
    x1, y1, x2, y2 = (int(round(v)) for v in box)
    mx, my = int((x2 - x1) * FLOOR_MARGIN), int((y2 - y1) * FLOOR_MARGIN)
    around = grid_mm[max(0, y1 - my):min(h, y2 + my), max(0, x1 - mx):min(w, x2 + mx), 2]
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    pts = grid_mm[y1:y2, x1:x2]
    if pts.shape[0] < 3 or pts.shape[1] < 3 or np.isfinite(around).sum() < MIN_POINTS:
        return None
    floor = np.nanpercentile(around, FLOOR_PERCENTILE)

    # NaN compares False, so a pixel without depth is never on the part and
    # a step to one is always an edge.
    with np.errstate(invalid="ignore"):
        on = pts[..., 2] > floor + MIN_HEIGHT_MM
        across = ~(np.linalg.norm(pts[:, 1:] - pts[:, :-1], axis=2) <= EDGE_MM)
        down = ~(np.linalg.norm(pts[1:] - pts[:-1], axis=2) <= EDGE_MM)
    on[:, 1:] &= ~across
    on[:, :-1] &= ~across
    on[1:] &= ~down
    on[:-1] &= ~down

    # The part is the piece that fills most of the middle third of the box.
    n, labels = cv2.connectedComponents(on.astype(np.uint8), connectivity=4)
    bh, bw = labels.shape
    middle = labels[bh // 3:bh - bh // 3, bw // 3:bw - bw // 3]
    counts = np.bincount(middle.ravel(), minlength=n)
    counts[0] = 0
    if counts.max() == 0:
        return None
    part = pts[labels == counts.argmax()]
    if len(part) < MIN_POINTS:
        return None

    top_z = float(np.percentile(part[:, 2], TOP_PERCENTILE))
    top = part[part[:, 2] >= top_z - TOP_BAND_MM, :2].astype(np.float32)
    if len(top) < MIN_POINTS:
        return None
    (cx, cy), _, _ = cv2.minAreaRect(top)
    return float(cx), float(cy), top_z
