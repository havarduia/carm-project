# model_training

YOLO26s component detector (capacitor, resistor, transformer), trained locally on
our Roboflow data plus the public PCB Component Detection dataset.

## Reproduce

```bash
# 1. Data (not in git)
unzip "carm capacitor test set.v8-v8.yolo26.zip" -d datasets/carm_v8      # Roboflow export
curl -L -o datasets/pcb_ninja.tar "<Dataset Ninja download link>"          # see below
mkdir -p datasets/pcb_ninja && tar -xf datasets/pcb_ninja.tar -C datasets/pcb_ninja

# 2. Environment - separate from ROS (ROS pins numpy<2, ultralytics needs 2.x)
python3 -m venv .venv
.venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
.venv/bin/pip install ultralytics

# 3. Build datasets/combined/, then train + test
python3 prepare_data.py
.venv/bin/python train.py
```

Outputs: `runs/yolo26s_combined/weights/best.pt`, `results.csv`, plots, and
`test_metrics.json`.

## Data

| | Our images (source photos) | Public PCB images |
| --- | --- | --- |
| Train | 107 (v9: 321 files, each photo x3; v8: 305 augmented files) | 1,296 |
| Val | 30 | - |
| Test | 15 | - |

- Our split is by source photo (stratified so resistors land in val and test), so no
  augmented copy of a test photo is trained on.
- Our Roboflow labels are mostly polygons; `prepare_data.py` converts them to boxes.
- Public data: [PCB Component Detection](https://datasetninja.com/pcb-component-detection)
  (CC0), from Kaggle `animeshkumarnayak/pcb-fault-detection`. Cap1-4 -> capacitor,
  Resistor -> resistor, Transformer -> transformer; MOSFET/MOV dropped.

## Results (2026-10-04)

**Use `runs/yolo26s_v9_1280/weights/best.pt`.**

| Run | Data | imgsz | Best epoch |
| --- | --- | --- | --- |
| `yolo26s_v9_1280` | v9 export (fit within 1280, no aug), ours x3 | 1280 | 74 of 104 |
| `yolo26s_combined` | v8 export (stretched 640, 3x aug) | 640 | 77 of 107 |

```bash
python3 prepare_data.py --ours carm_v9 --out combined_v9 --repeat 3
.venv/bin/python train.py --data combined_v9 --name yolo26s_v9_1280 --imgsz 1280 --batch 8
```

Both models scored on the same 15 clean v9 test photos (290 objects):

| Class | Instances | 640 mAP@50 | 1280 mAP@50 | 640 mAP@50-95 | 1280 mAP@50-95 | 1280 P | 1280 R |
| --- | --- | --- | --- | --- | --- | --- | --- |
| All | 290 | 0.77 | 0.84 | 0.44 | 0.60 | 0.78 | 0.85 |
| Capacitor | 253 | 0.84 | 0.91 | 0.53 | 0.70 | 0.85 | 0.88 |
| Resistor | 8 | 0.75 | 0.81 | 0.30 | 0.49 | 0.76 | 0.88 |
| Transformer | 29 | 0.72 | 0.81 | 0.49 | 0.60 | 0.74 | 0.78 |

1280 model: best F1 (0.81) is at confidence 0.50, matching `CONF_THRESHOLD` in
`detection_model/yolo_model.py`. Validation (30 images, 821 objects): mAP@50 0.85,
mAP@50-95 0.63.
