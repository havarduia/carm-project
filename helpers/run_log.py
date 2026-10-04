"""Per-run log of every snapshot and pick attempt, written under logs/.

One folder per run of main.py:

    logs/2026-10-04_183000/
        picks.csv           one row per pick attempt
        snapshots.csv       one row per snapshot, with the operator's counts
        snapshot_01_raw.jpg, snapshot_01_detections.jpg, ...
"""

import csv
import os
import time

import cv2

LOG_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")


class RunLog:
    def __init__(self, root=LOG_ROOT):
        self.dir = os.path.join(root, time.strftime("%Y-%m-%d_%H%M%S"))
        os.makedirs(self.dir, exist_ok=True)
        self._writers = {}

    def write(self, table, row):
        """Append `row` (a dict) to <table>.csv. The first row sets the columns."""
        if table not in self._writers:
            f = open(os.path.join(self.dir, table + ".csv"), "w", newline="")
            writer = csv.DictWriter(f, fieldnames=list(row))
            writer.writeheader()
            self._writers[table] = (f, writer)
        f, writer = self._writers[table]
        writer.writerow(row)
        # Flushed per row, so an e-stop or a crash loses nothing already done.
        f.flush()

    def save_image(self, name, image):
        cv2.imwrite(os.path.join(self.dir, name), image)

    def close(self):
        for f, _ in self._writers.values():
            f.close()
