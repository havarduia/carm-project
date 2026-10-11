# model_training

Training of `yolo26s_v9_1280`, the component detector the robot runs. It finds three
classes: capacitor, resistor and transformer.

- What the model is, its settings and its known limitations:
  [`yolo26s_v9_1280.md`](yolo26s_v9_1280.md).
- Its test scores, curves, confusion matrix and dataset breakdown, as plots and tables:
  [`../results/yolo26s_v9_1280/`](../results/yolo26s_v9_1280/README.md).

## Files

| File | Does |
| --- | --- |
| `prepare_data.py` | Builds the training dataset from our Roboflow export and the public PCB dataset |
| `train.py` | Trains the model and scores it on the test split |
| `yolo26s_v9_1280.md` | Model card |

The datasets, the training runs, the weights and the venv are not in git.

## Reproduce

```bash
# 1. Data
unzip "carm capacitor test set.v9i.yolo26.zip" -d datasets/carm_v9     # our Roboflow export
# public dataset: download the archive from the Dataset Ninja page linked under Data
mkdir -p datasets/pcb_ninja && tar -xf datasets/pcb_ninja.tar -C datasets/pcb_ninja

# 2. Environment, separate from the robot's venv
python3 -m venv .venv
.venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
.venv/bin/pip install ultralytics

# 3. Build datasets/combined_v9/, then train and test
python3 prepare_data.py --ours carm_v9 --out combined_v9 --repeat 3
.venv/bin/python train.py --data combined_v9 --name yolo26s_v9_1280 --imgsz 1280 --batch 8
```

The run takes about 55 minutes on an RTX 5070 and writes `runs/yolo26s_v9_1280/`:
`weights/best.pt`, the per-epoch log `results.csv`, and `test_metrics.json`. Copy
`best.pt` to `../detection_model/weights/yolo26s_v9_1280.pt` for the robot to use it.

## Data

| Split | Images | Source |
| --- | --- | --- |
| Train | 107 of our photos, each included three times, and 1,296 public images | Both |
| Validation | 30 | Our photos |
| Test | 15 | Our photos |

- **Our photos** are version 9 of the Roboflow project
  [carm capacitor test set](https://universe.roboflow.com/carm-yitb6/carm-capacitor-test-set/dataset/9):
  152 photos, exported fit within 1280 pixels with no augmentation. The labels are mostly
  polygons, which `prepare_data.py` converts to boxes.
- **Public images** are [PCB Component Detection](https://datasetninja.com/pcb-component-detection)
  from Dataset Ninja (CC0). Its four capacitor classes map to capacitor, Resistor to
  resistor and Transformer to transformer; MOSFET and MOV boxes are dropped. They are
  used for training only, so validation and test say how the model does on our own rig.
- **The split** of our photos is by source photo and stratified so that resistors land
  in validation and test. No photo is in more than one split. Our training photos are
  repeated three times so that the public images do not drown them out.

## Result

Test split, 15 photos with 290 labelled objects, inference size 1280:

| Class | Objects | Precision | Recall | mAP@50 | mAP@50-95 |
| --- | --- | --- | --- | --- | --- |
| All | 290 | 0.78 | 0.85 | 0.84 | 0.60 |
| Capacitor | 253 | 0.85 | 0.88 | 0.91 | 0.70 |
| Resistor | 8 | 0.76 | 0.88 | 0.81 | 0.49 |
| Transformer | 29 | 0.74 | 0.78 | 0.80 | 0.60 |

Validation, 30 photos with 821 objects: mAP@50 0.85, mAP@50-95 0.63.

The best F1 is at confidence 0.50, which is the threshold the robot uses
(`CONF_THRESHOLD` in `detection_model/yolo_model.py`).
