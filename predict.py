import os
import sys
import glob
import json
import time
from typing import List, Tuple

import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
from PIL import Image
import pandas as pd

# ============ Path Configuration ============
# Data source (inference only)
XOZ_DIR = r"D:\Ti\Py_mmWave_Roformer\Dataset\squat.test\pHistBytes_clustered_voxel\pHistBytes_clustered_voxel_XOZ"
YOZ_DIR = r"D:\Ti\Py_mmWave_Roformer\Dataset\squat.test\pHistBytes_clustered_voxel\pHistBytes_clustered_voxel_YOZ"

# Model and code paths
ROPE_INFORMER_DIR = r"D:\Ti\Py_mmWave_Roformer\rope_informer"  # Should contain Exp_Informer and model code
CHECKPOINT_PATH = r"D:\Ti\Py_mmWave_Roformer\model_checkpoint\checkpoint0914.pth"

# Output directory
OUTPUT_DIR = r"D:\Ti\Py_mmWave_Roformer\inference_outputs"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ============ Parameters and Class Mapping ============
behavior_mapping = {
    "stationary": 0,
    "run": 1,
    "squat": 2,
    "stand": 3,
    "walk": 4,
}
id2label = {v: k for k, v in behavior_mapping.items()}

# View image sizes
image_sizes = {
    "xoz": (25, 25),  # W, H
    "yoz": (25, 15),
}

# Sliding window and sequence length (must match Kaggle training setup)
WINDOW_SIZE = 3
SEQ_LEN = 3000  # 3 * (25*25 + 25*15) = 3 * (625 + 375) = 3000
BATCH_SIZE = 32

# Whether ground-truth labels are available
# (Your run.test set has no multi-class annotation; if all samples are known as "run", set this to True)
has_labels = True
default_label_idx = behavior_mapping["squat"]

# ============ Import rope_informer Code ============
if ROPE_INFORMER_DIR not in sys.path:
    sys.path.append(ROPE_INFORMER_DIR)

# Assume Exp_Informer provides the same interface as the Kaggle version
try:
    from rope_informer import Exp_Informer
except Exception as e:
    raise ImportError(
        f"Failed to import Exp_Informer from {ROPE_INFORMER_DIR}. "
        f"Please check the path and module structure. Original error: {e}"
    )

# ============ Dataset Definition (Inference Only) ============
class SlidingWindowFolderDataset(Dataset):
    """
    Read images from two folders (XOZ / YOZ), sort them by filename,
    and apply sliding windows independently for each view.

    Each sample is generated from a single view only (consistent with the
    Kaggle training logic: windowed samples from XOZ or YOZ separately).

    If you want to concatenate temporally aligned XOZ and YOZ views into
    a single sample, please define the alignment rule first and modify this class.
    """

    def __init__(
        self,
        xoz_dir: str,
        yoz_dir: str,
        window_size: int = 3,
        seq_len: int = 3000,
        label_idx: int = None
    ):
        self.window_size = window_size
        self.seq_len = seq_len
        self.samples: List[Tuple[List[str], str]] = []  # (paths, view)
        self.labels: List[int] = []

        self._collect(xoz_dir, "xoz", label_idx)
        self._collect(yoz_dir, "yoz", label_idx)

    def _collect(self, folder: str, view: str, label_idx: int):
        if not folder or not os.path.isdir(folder):
            return

        exts = (".png", ".jpg", ".jpeg", ".bmp")
        imgs = [p for p in glob.glob(os.path.join(folder, "*")) if p.lower().endswith(exts)]
        imgs.sort()

        if len(imgs) < self.window_size:
            return

        for i in range(len(imgs) - self.window_size + 1):
            win = imgs[i:i + self.window_size]
            self.samples.append((win, view))
            self.labels.append(label_idx if label_idx is not None else -1)

    def __len__(self):
        return len(self.samples)

    def _view_resize(self, img: Image.Image, view: str) -> Image.Image:
        size = image_sizes["xoz"] if view == "xoz" else image_sizes["yoz"]
        return img.resize(size)

    def __getitem__(self, idx):
        paths, view = self.samples[idx]

        window_arrays = []
        for p in paths:
            img = Image.open(p).convert("L")
            img = self._view_resize(img, view)
            arr = np.array(img, dtype=np.float32).flatten()
            window_arrays.append(arr)

        seq = np.concatenate(window_arrays, axis=0)

        # Pad or truncate to seq_len
        if len(seq) < self.seq_len:
            pad = self.seq_len - len(seq)
            seq = np.pad(seq, (0, pad), mode="constant")
        elif len(seq) > self.seq_len:
            seq = seq[:self.seq_len]

        x = torch.from_numpy(seq).float()  # shape: [seq_len]
        y = torch.tensor(self.labels[idx], dtype=torch.long) if self.labels[idx] >= 0 else torch.tensor(-1)

        meta = {
            "paths": paths,
            "view": view
        }
        return x, y, meta

# ============ Args Definition (Must Match Training Configuration) ============
class Args:
    def __init__(self):
        self.model = 'informer'
        self.data = 'Classification'
        self.root_path = ''
        self.enc_in = 1
        self.d_model = 64
        self.d_ff = 256
        self.train_epochs = 1
        self.batch_size = BATCH_SIZE
        self.seq_len = SEQ_LEN
        self.output_path = OUTPUT_DIR
        self.checkpoints = OUTPUT_DIR  # No checkpoint writing during inference
        self.test_ratio = 0.2
        self.n_heads = 8
        self.has_rope = True
        self.features = 'M'
        self.target = 'OT'
        self.freq = 'h'
        self.label_len = 48
        self.pred_len = 24
        self.dec_in = 1
        self.c_out = 5
        self.e_layers = 2
        self.d_layers = 1
        self.s_layers = '3,2,1'
        self.factor = 5
        self.padding = 0
        self.distil = True
        self.dropout = 0.05
        self.attn = 'prob'
        self.embed = 'timeF'
        self.activation = 'gelu'
        self.output_attention = False
        self.do_predict = True
        self.mix = True
        self.cols = None
        self.num_workers = 0
        self.itr = 1
        self.patience = 2
        self.learning_rate = 1e-4
        self.des = 'inference'
        self.loss = 'mse'
        self.lradj = 'type1'
        self.use_amp = False
        self.inverse = False
        self.use_gpu = torch.cuda.is_available()
        self.gpu = 0
        self.use_multi_gpu = False
        self.devices = '0'

args = Args()

# ============ Prepare Dataset ============
label_idx = default_label_idx if has_labels else None
dataset = SlidingWindowFolderDataset(XOZ_DIR, YOZ_DIR, WINDOW_SIZE, SEQ_LEN, label_idx)

if len(dataset) == 0:
    raise RuntimeError(
        "No sufficient images were found to construct sliding-window samples. "
        "Please check directory paths and file extensions."
    )

def collate_with_meta(batch):
    """
    Custom collate function that preserves per-sample metadata.
    """
    xs, ys, metas = [], [], []
    for x, y, meta in batch:
        xs.append(x)
        ys.append(y)
        metas.append(meta)

    xs = torch.stack(xs, dim=0)  # [B, seq_len]
    ys = torch.stack(ys, dim=0) if isinstance(ys[0], torch.Tensor) else torch.tensor(ys, dtype=torch.long)
    return xs, ys, metas

loader = DataLoader(
    dataset,
    batch_size=BATCH_SIZE,
    shuffle=False,
    num_workers=0,
    collate_fn=collate_with_meta
)

# ============ Custom Experiment Class (Inference Only) ============
class InferenceExp(Exp_Informer):
    def _get_data(self, flag):
        # Keep Exp_Informer interface compatibility; return custom dataset and loader
        return dataset, loader

# ============ Create Experiment, Build Model, and Load Checkpoint ============
exp = InferenceExp(args)
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

model = getattr(exp, "model", None)
if model is None:
    # If Exp_Informer does not build the model in __init__, try to build it manually
    try:
        model = exp._build_model()
        exp.model = model
    except Exception as e:
        raise RuntimeError(f"Failed to build model. Please check Exp_Informer implementation. Error: {e}")

model = model.to(device)
model.eval()

def load_checkpoint_into_model(model: torch.nn.Module, ckpt_path: str):
    if not os.path.isfile(ckpt_path):
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    state = torch.load(ckpt_path, map_location=device)

    # Support multiple checkpoint formats
    if isinstance(state, dict) and "state_dict" in state:
        state_dict = state["state_dict"]
    elif isinstance(state, dict) and "model_state_dict" in state:
        state_dict = state["model_state_dict"]
    elif isinstance(state, dict):
        state_dict = state
    else:
        raise ValueError("Unknown checkpoint format. Expected state_dict or model_state_dict.")

    # Remove DataParallel prefix if present
    new_state = {}
    for k, v in state_dict.items():
        nk = k.replace("module.", "") if k.startswith("module.") else k
        new_state[nk] = v

    missing, unexpected = model.load_state_dict(new_state, strict=False)
    if missing:
        print(f"Warning: missing parameters: {missing}")
    if unexpected:
        print(f"Warning: unexpected parameters: {unexpected}")

load_checkpoint_into_model(model, CHECKPOINT_PATH)

# ============ Inference ============
all_preds = []
all_probs = []
all_labels = []
all_views = []
all_first_paths = []

softmax = torch.nn.Softmax(dim=-1)

start = time.time()
with torch.no_grad():
    for x, y, meta in loader:
        x = x.to(device)

        # Adjust according to Exp_Informer forward interface
        logits = model(x) if callable(model) else model.forward(x)
        if isinstance(logits, tuple):
            logits = logits[0]

        probs = softmax(logits)
        preds = torch.argmax(probs, dim=-1)

        all_preds.append(preds.cpu().numpy())
        all_probs.append(probs.cpu().numpy())

        if has_labels:
            all_labels.append(y.numpy())
        else:
            all_labels.append(np.full((x.size(0),), -1, dtype=np.int64))

        # Record metadata
        # meta is a list of dictionaries (one per sample)
        # Use the first frame path as the sample identifier
        for sample_meta in meta:
            all_views.append(sample_meta["view"])
            all_first_paths.append(sample_meta["paths"][0])

elapsed = time.time() - start

all_preds = np.concatenate(all_preds, axis=0)
all_probs = np.concatenate(all_probs, axis=0)
all_labels = np.concatenate(all_labels, axis=0)

# ============ Accuracy Evaluation (only if has_labels=True) ============
acc = None
if has_labels and np.all(all_labels >= 0):
    acc = float((all_preds == all_labels).mean())
    print(
        f"Inference completed. Samples: {len(dataset)}, "
        f"Accuracy: {acc:.4f}, Time: {elapsed:.2f}s"
    )
else:
    print(
        f"Inference completed. Samples: {len(dataset)}, "
        f"Time: {elapsed:.2f}s"
    )

# ============ Save Results ============
pred_labels = [id2label[int(i)] for i in all_preds]
max_probs = all_probs.max(axis=1)

df = pd.DataFrame({
    "sample_id": list(range(len(pred_labels))),
    "first_frame_path": all_first_paths,
    "view": all_views,
    "pred_label": pred_labels,
    "pred_index": all_preds.astype(int),
    "pred_confidence": max_probs
})

if has_labels:
    df["true_index"] = all_labels.astype(int)
    df["true_label"] = [
        id2label[i] if i in id2label else "unknown"
        for i in df["true_index"]
    ]

csv_path = os.path.join(OUTPUT_DIR, "inference_results.csv")
df.to_csv(csv_path, index=False, encoding="utf-8-sig")

summary = {
    "samples": int(len(dataset)),
    "has_labels": bool(has_labels),
    "accuracy": float(acc) if acc is not None else None,
    "elapsed_sec": float(elapsed),
    "classes": id2label
}

with open(os.path.join(OUTPUT_DIR, "summary.json"), "w", encoding="utf-8") as f:
    json.dump(summary, f, ensure_ascii=False, indent=2)

print(
    f"Results saved to:\n"
    f"- {csv_path}\n"
    f"- {os.path.join(OUTPUT_DIR, 'summary.json')}"
)
