"""Build datasets/combined/ from our Roboflow export + the public PCB dataset.

Our data (carm_v8, Roboflow YOLO export):
  The v8 export holds 140 train sources x 3 augmented copies, plus 10 valid and
  2 test images. That split is too small to measure anything, so it is redone
  here by *source image*: every augmented copy of a source stays in the same
  split, otherwise near-identical images would leak from train into val/test.
  Val/test keep one copy per source.

Public data (pcb_ninja, Supervisely JSON, CC0):
  PCB Component Detection, 1,410 top-down PCB photos. All of it goes into
  train only - val and test are our own images, so the reported numbers are
  about our rig, not someone else's boards.
  Cap1-4 -> capacitor, Resistor/Resestor -> resistor, Transformer -> transformer.
  MOSFET and Mov boxes are dropped, which teaches the model they are background
  (we do not want them picked as capacitors). Images with no boxes at all are
  unlabelled in the source and are skipped.
"""
import argparse
import json
import random
import re
import shutil
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent / "datasets"
OURS = ROOT / "carm_v8"
PUBLIC = ROOT / "pcb_ninja"
OUT = ROOT / "combined"
REPEAT = 1

NAMES = ["capacitor", "resistor", "transformer"]
PUBLIC_MAP = {
    "Cap1": 0, "Cap2": 0, "Cap3": 0, "Cap4": 0,
    "Resistor": 1, "Resestor": 1,
    "Transformer": 2,
}

# Share of our *source* images held out for validation and test.
VAL_SHARE, TEST_SHARE = 0.20, 0.10
SEED = 0


def source_of(path):
    """'12_png.rf.<hash>.jpg' -> '12_png': every augmented copy shares this."""
    return re.sub(r"\.rf\.[0-9a-f]+$", "", path.stem)


def to_box_row(line):
    """YOLO row -> 'cls cx cy w h'. Polygon rows become their bounding box.

    Most of our Roboflow labels are polygons, a few are boxes, and many files
    mix both. Ultralytics discards any file that mixes the two, so everything
    is normalised to boxes here.
    """
    cls, *vals = line.split()
    vals = [float(v) for v in vals]
    if len(vals) == 4:
        cx, cy, w, h = vals
        xs, ys = [cx - w / 2, cx + w / 2], [cy - h / 2, cy + h / 2]
    else:
        xs, ys = vals[0::2], vals[1::2]
    # Roboflow's rotation augmentation leaves some points up to ~3% outside
    # the frame, which Ultralytics rejects. Clip to the image.
    x1, x2 = max(0.0, min(xs)), min(1.0, max(xs))
    y1, y2 = max(0.0, min(ys)), min(1.0, max(ys))
    return f"{cls} {(x1 + x2) / 2:.6f} {(y1 + y2) / 2:.6f} {x2 - x1:.6f} {y2 - y1:.6f}"


def copy_pair(img, label, split, name):
    shutil.copy(img, OUT / split / "images" / f"{name}{img.suffix}")
    rows = [to_box_row(l) for l in label.read_text().splitlines() if l.strip()] if label.exists() else []
    (OUT / split / "labels" / f"{name}.txt").write_text("\n".join(rows))


def split_ours():
    groups = defaultdict(list)
    for split in ("train", "valid", "test"):
        for img in sorted((OURS / split / "images").iterdir()):
            groups[source_of(img)].append(img)

    # Resistors appear in only a handful of sources, so a plain shuffle can
    # leave none in test. Split resistor sources and the rest separately.
    def has_resistor(src):
        img = groups[src][0]
        label = img.parent.parent / "labels" / f"{img.stem}.txt"
        return any(line.split()[:1] == ["1"] for line in label.read_text().splitlines())

    rng = random.Random(SEED)
    assign = {}
    for stratum in (True, False):
        sources = sorted(s for s in groups if has_resistor(s) == stratum)
        rng.shuffle(sources)
        n_test = max(1, round(len(sources) * TEST_SHARE))
        n_val = max(1, round(len(sources) * VAL_SHARE))
        assign.update({s: "test" for s in sources[:n_test]})
        assign.update({s: "val" for s in sources[n_test:n_test + n_val]})
        assign.update({s: "train" for s in sources[n_test + n_val:]})

    counts = defaultdict(lambda: [0, 0])
    for src, imgs in groups.items():
        split = assign[src]
        # Train keeps every augmented copy (x REPEAT, to hold our share of the
        # mix against the public set); val/test one copy per source.
        for img in (imgs if split == "train" else imgs[:1]):
            label = img.parent.parent / "labels" / f"{img.stem}.txt"
            for r in range(REPEAT if split == "train" else 1):
                copy_pair(img, label, split, f"carm_{img.stem}" + (f"_r{r}" if r else ""))
                counts[split][1] += 1
        counts[split][0] += 1
    return counts


def convert_public():
    n_img = n_box = 0
    for split in ("train", "validation", "test"):
        for ann_path in sorted((PUBLIC / split / "ann").iterdir()):
            ann = json.loads(ann_path.read_text())
            if not ann["objects"]:
                continue
            w, h = ann["size"]["width"], ann["size"]["height"]
            lines = []
            for obj in ann["objects"]:
                cls = PUBLIC_MAP.get(obj["classTitle"])
                if cls is None:
                    continue
                (x1, y1), (x2, y2) = obj["points"]["exterior"]
                x1, x2 = sorted((x1, x2))
                y1, y2 = sorted((y1, y2))
                lines.append(
                    f"{cls} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} "
                    f"{(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}"
                )
            img = PUBLIC / split / "img" / ann_path.stem  # ann is '<image name>.json'
            name = f"pcb_{Path(ann_path.stem).stem}"
            shutil.copy(img, OUT / "train" / "images" / f"{name}{img.suffix}")
            (OUT / "train" / "labels" / f"{name}.txt").write_text("\n".join(lines))
            n_img += 1
            n_box += len(lines)
    return n_img, n_box


def main():
    global OURS, OUT, REPEAT
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ours", default="carm_v8", help="our Roboflow export, under datasets/")
    parser.add_argument("--out", default="combined", help="output folder, under datasets/")
    parser.add_argument("--repeat", type=int, default=1,
                        help="copies of each of our train images (use 3 for an unaugmented export)")
    args = parser.parse_args()
    OURS, OUT, REPEAT = ROOT / args.ours, ROOT / args.out, args.repeat

    if OUT.exists():
        shutil.rmtree(OUT)
    for split in ("train", "val", "test"):
        (OUT / split / "images").mkdir(parents=True)
        (OUT / split / "labels").mkdir(parents=True)

    counts = split_ours()
    n_img, n_box = convert_public()

    (OUT / "data.yaml").write_text(
        f"path: {OUT}\ntrain: train/images\nval: val/images\ntest: test/images\n"
        f"names: {dict(enumerate(NAMES))}\n"
    )

    for split in ("train", "val", "test"):
        src, imgs = counts[split]
        print(f"ours {split:5s}: {src:3d} source images -> {imgs:3d} files")
    print(f"public train: {n_img} images, {n_box} boxes kept")


if __name__ == "__main__":
    main()
