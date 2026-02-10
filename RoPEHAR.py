import sys
import os


# Set the model, data and output paths
model_path = '/home/jiacheng008/Py_mmWave_Roformer/rope_informer'
checkpoints = '/home/jiacheng008/Py_mmWave_Roformer/checkpoints/'
output_path = '/home/jiacheng008/Py_mmWave_Roformer/test_results/'

# Ensure that the directory exists
os.makedirs(checkpoints, exist_ok=True)
os.makedirs(output_path, exist_ok=True)

# Add the model path to sys.path
sys.path.append(os.path.dirname(model_path))

print("Environment setup completed")

# Import required libraries
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
from sklearn.model_selection import train_test_split
import argparse
import matplotlib.pyplot as plt

print("Library import completed")

# Definition of Data Paths and Category Mapping
data_paths = {
    "stationary": {
        "xoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/stationary/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/stationary/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "run": {
        "xoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/run/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/run/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "squat": {
        "xoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/squat/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/squat/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "stand": {
        "xoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/stand/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/stand/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "walk": {
        "xoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/walk/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "/home/jiacheng008/Py_mmWave_Roformer/Dataset/walk/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    }
}

# Behavior category mapping
behavior_mapping = {
    "stationary": 0,
    "run": 1,
    "squat": 2,
    "stand": 3,
    "walk": 4
}

# Definition of image sizes
image_sizes = {
    "xoz": (25, 25),
    "yoz": (25, 15)
}

print("Data paths and category definitions completed")


# Custom dataset class (with sliding window support)
class BehaviorDataset(Dataset):
    def __init__(self, data_paths, window_size=3, seq_len=15000, is_train=True):
        self.window_size = window_size
        self.seq_len = seq_len
        self.data = []
        self.labels = []

        print("Start collecting data...")
        for behavior, paths in data_paths.items():
            label = behavior_mapping[behavior]
            print(f"Processing behavior: {behavior} (label: {label})")

            # Handle the XOZ view
            xoz_path = paths["xoz"]
            if os.path.exists(xoz_path):
                xoz_images = []
                for img_name in sorted(os.listdir(xoz_path)):
                    if img_name.endswith(('.png', '.jpg', '.jpeg')):
                        xoz_images.append(os.path.join(xoz_path, img_name))

                # Create sliding-window samples for the XOZ view
                for i in range(len(xoz_images) - window_size + 1):
                    self.data.append((xoz_images[i:i + window_size], label, "xoz"))
                print(f"  XOZ view: added {len(xoz_images) - window_size + 1} samples")

            # Handle the YOZ view
            yoz_path = paths["yoz"]
            if os.path.exists(yoz_path):
                yoz_images = []
                for img_name in sorted(os.listdir(yoz_path)):
                    if img_name.endswith(('.png', '.jpg', '.jpeg')):
                        yoz_images.append(os.path.join(yoz_path, img_name))

                # Create sliding-window samples for the YOZ view
                for i in range(len(yoz_images) - window_size + 1):
                    self.data.append((yoz_images[i:i + window_size], label, "yoz"))
                print(f"  YOZ view: added {len(yoz_images) - window_size + 1} samples")

        print(f"Collected {len(self.data)} samples in total")

        if len(self.data) == 0:
            print("Warning: no data samples were found!")
            return

        # Split into training and testing sets
        try:
            train_data, test_data = train_test_split(
                self.data, test_size=0.2, random_state=42, stratify=[d[1] for d in self.data]
            )

            self.data = train_data if is_train else test_data
            print(f"{'Training' if is_train else 'Test'} set size: {len(self.data)}")
        except Exception as e:
            print(f"Error while splitting the dataset: {e}")

            train_data, test_data = train_test_split(
                self.data, test_size=0.2, random_state=42
            )
            self.data = train_data if is_train else test_data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        img_paths, label, view = self.data[idx]
        window_images = []

        # Load all images within the window
        for img_path in img_paths:
            img = Image.open(img_path).convert('L')

            if view == "xoz":
                img = img.resize(image_sizes["xoz"])
            else:
                img = img.resize(image_sizes["yoz"])

            img_array = np.array(img).flatten()
            window_images.append(img_array)

        # Combine all images in the window into a single sequence
        sequence = np.concatenate(window_images)

        if len(sequence) < self.seq_len:
            pad_width = self.seq_len - len(sequence)
            sequence = np.pad(sequence, (0, pad_width), mode='constant')
        elif len(sequence) > self.seq_len:
            sequence = sequence[:self.seq_len]

        return torch.FloatTensor(sequence), torch.tensor(label, dtype=torch.long)


print("Custom dataset class definition completed")

# Create data loaders
# Sequence length calculation:
# Each sample has 3 frames.
# Flattened length per frame:
#   XOZ: 25*25 = 625
#   YOZ: 25*15 = 375
# Total sequence length = 3 * (625 + 375) = 3000
seq_len = 3000

print("Creating training dataset...")
train_dataset = BehaviorDataset(data_paths, window_size=3, seq_len=seq_len, is_train=True)
print("Creating test dataset...")
test_dataset = BehaviorDataset(data_paths, window_size=3, seq_len=seq_len, is_train=False)

train_loader = DataLoader(train_dataset, batch_size=8, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=8, shuffle=False)

print(f"Training set size: {len(train_dataset)}")
print(f"Test set size: {len(test_dataset)}")
print("Data loaders created successfully")

# Parameter settings
class Args:
    def __init__(self):
        self.model = 'informer'
        self.data = 'Classification'
        self.root_path = ''
        self.enc_in = 1
        self.d_model = 64
        self.d_ff = 256
        self.train_epochs = 20
        self.batch_size = 8
        self.seq_len = seq_len  # Use the computed sequence length
        self.output_path = output_path
        self.checkpoints = checkpoints
        self.test_ratio = 0.2
        self.n_heads = 8
        self.has_rope = True
        self.features = 'M'
        self.target = 'OT'
        self.freq = 'h'
        self.label_len = 48
        self.pred_len = 24
        self.dec_in = 1
        self.c_out = 5  # 5 behavior classes
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
        self.do_predict = False
        self.mix = True
        self.cols = None
        self.num_workers = 0
        self.itr = 1
        self.patience = 2
        self.learning_rate = 0.0001
        self.des = 'test'
        self.loss = 'mse'
        self.lradj = 'type1'
        self.use_amp = False
        self.inverse = False
        self.use_gpu = True if torch.cuda.is_available() else False
        self.gpu = 0
        self.use_multi_gpu = False
        self.devices = '0,1,2,3'


args = Args()

print("Parameter settings completed")
print(f"Do you use a GPU?: {args.use_gpu}")

# Import the model and set up the experiment
try:
    from rope_informer import Exp_Informer
    print("Successfully imported rope_informer")
except ImportError as e:
    print(f"Failed to import rope_informer: {e}")
    sys.exit(1)


# Modify the Exp_Informer class to use the new data loaders
class CustomExp_Informer(Exp_Informer):
    def _get_data(self, flag):
        if flag == 'test':
            return test_dataset, test_loader
        else:
            return train_dataset, train_loader


# Set up the experiment
setting = '{}_{}_ft{}_sl{}_ll{}_pl{}_dm{}_nh{}_el{}_dl{}_df{}_at{}_fc{}_eb{}_dt{}_mx{}_{}_{}'.format(
    args.model, args.data, args.features,
    args.seq_len, args.label_len, args.pred_len,
    args.d_model, args.n_heads, args.e_layers, args.d_layers, args.d_ff, args.attn, args.factor,
    args.embed, args.distil, args.mix, args.des, 1)

exp = CustomExp_Informer(args)

print("Model import and experiment setup completed")

print('>>>>>>>start training : {}>>>>>>>>>>>>>>>>>>>>>>>>>>'.format(setting))
exp.train(setting)

print("Model training completed")

# Model testing
print('>>>>>>>testing : {}<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<'.format(setting))
exp.test(setting)

torch.cuda.empty_cache()

print("Model testing completed")

# Results visualization
try:
    truth = np.load(os.path.join(args.output_path, setting, "true.npy"))
    preds = np.load(os.path.join(args.output_path, setting, "pred.npy"))

    # Compute the total count per class and the count of correct predictions per class
    bin_counts = np.bincount(truth, minlength=5)
    correct_counts = np.bincount(truth[preds == truth], minlength=5)

    # Compute overall accuracy
    accuracy = np.sum(preds == truth) / len(truth)
    print(f"Overall accuracy rate: {accuracy:.4f}")

    # Plot histogram
    categories = ['stationary', 'run', 'squat', 'stand', 'walk']
    plt.figure(figsize=(10, 6))
    plt.bar(categories, bin_counts, width=0.5, align='center', alpha=0.8, label='truth', color='purple')
    plt.bar(categories, correct_counts, width=0.5, align='edge', alpha=1.0, label='correct', color='lightgreen')
    plt.xlabel('Behavior Categories')
    plt.ylabel('Count')
    plt.title('Classification Results for 5 Human Behaviors (3-frame window)')
    plt.legend()
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig('/home/jiacheng008/Py_mmWave_Roformer/behavior_classification_results_window.png')
    print("Result visualization completed and saved as an image.")
except Exception as e:
    print(f"Result visualization failed: {e}")
