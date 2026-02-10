import os
import numpy as np
import re
from tqdm import tqdm
import torch
from torch.utils.data import Dataset, DataLoader
from skimage.transform import resize
from PIL import Image
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split


def natural_sort_key(s):
    """
    Natural sorting key function to ensure numbers are sorted
    according to their numerical value rather than lexicographically.
    """
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('(\d+)', s)]


def load_image_pairs(class_folders, window_size=5):
    """
    Load XOZ and YOZ image pairs from multiple action classes.

    Args:
        class_folders (dict):
            A dictionary where keys are class names and values are
            dictionaries containing paths to XOZ and YOZ folders.
        window_size (int):
            Sliding window size for temporal sequences.

    Returns:
        X (np.ndarray):
            Processed image sequences with shape
            (n_samples, window_size, 2, 25, 25)
        y (np.ndarray):
            Corresponding class labels.
        class_names (list):
            List of class names.
    """
    X = []
    y = []

    # Assign a numeric label to each class
    class_names = list(class_folders.keys())
    class_labels = {class_name: idx for idx, class_name in enumerate(class_names)}

    # Process data for each class
    for class_name, folders in class_folders.items():
        print(f"Processing class: {class_name}")

        # Get XOZ and YOZ folder paths
        xoz_folder = folders['xoz']
        yoz_folder = folders['yoz']

        # Get all image files and sort them naturally
        xoz_files = [f for f in os.listdir(xoz_folder) if f.endswith('.png') and '_XOZ' in f]
        yoz_files = [f for f in os.listdir(yoz_folder) if f.endswith('.png') and '_YOZ' in f]

        xoz_files.sort(key=natural_sort_key)
        yoz_files.sort(key=natural_sort_key)

        # Ensure the number of files is consistent
        min_files = min(len(xoz_files), len(yoz_files))
        xoz_files = xoz_files[:min_files]
        yoz_files = yoz_files[:min_files]

        print(f"  Found {min_files} image pairs")

        # Load and process image pairs
        image_pairs = []
        for xoz_file, yoz_file in tqdm(
            zip(xoz_files, yoz_files),
            total=min_files,
            desc=f"Processing {class_name} images"
        ):
            # Extract numeric indices from filenames to verify matching
            xoz_num = re.findall(r'\d+', xoz_file)[0]
            yoz_num = re.findall(r'\d+', yoz_file)[0]

            # Ensure file indices match
            if xoz_num != yoz_num:
                print(f"Warning: file mismatch - {xoz_file} vs {yoz_file}")
                continue

            # Load XOZ image (25x25)
            xoz_img = Image.open(os.path.join(xoz_folder, xoz_file))
            xoz_array = np.array(xoz_img.convert('L'))  # Convert to grayscale

            # Load YOZ image (15x25)
            yoz_img = Image.open(os.path.join(yoz_folder, yoz_file))
            yoz_array = np.array(yoz_img.convert('L'))  # Convert to grayscale

            # Resize YOZ image to 25x25
            yoz_resized = resize(
                yoz_array, (25, 25),
                preserve_range=True
            ).astype(np.uint8)

            # Stack the two projections into a 2-channel image
            two_channel_img = np.stack([xoz_array, yoz_resized], axis=0)
            image_pairs.append(two_channel_img)

        # Create sliding windows
        for i in range(len(image_pairs) - window_size + 1):
            window = image_pairs[i:i + window_size]
            X.append(np.array(window))
            y.append(class_labels[class_name])

    return np.array(X), np.array(y), class_names


# Define dataset folder paths
class_folders = {
    "stationary": {
        "xoz": "D:/Ti/Py_mmWave_Roformer/Dataset/stationary.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "D:/Ti/Py_mmWave_Roformer/Dataset/stationary.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "run": {
        "xoz": "D:/Ti/Py_mmWave_Roformer/Dataset/run.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "D:/Ti/Py_mmWave_Roformer/Dataset/run.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "squat": {
        "xoz": "D:/Ti/Py_mmWave_Roformer/Dataset/squat.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "D:/Ti/Py_mmWave_Roformer/Dataset/squat.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "stand": {
        "xoz": "D:/Ti/Py_mmWave_Roformer/Dataset/stand.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "D:/Ti/Py_mmWave_Roformer/Dataset/stand.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    },
    "walk": {
        "xoz": "D:/Ti/Py_mmWave_Roformer/Dataset/walk.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_XOZ",
        "yoz": "D:/Ti/Py_mmWave_Roformer/Dataset/walk.test/pHistBytes_clustered_voxel/pHistBytes_clustered_voxel_YOZ"
    }
}

# Load and process image data
window_size = 5
X, y, class_names = load_image_pairs(class_folders, window_size=window_size)

print(f"Dataset shape: X={X.shape}, y={y.shape}")
print(f"Class distribution: {np.bincount(y)}")
print(f"Class names: {class_names}")


def visualize_samples(X, y, class_names, num_samples=5):
    """
    Visualize randomly selected samples from the dataset.
    """
    fig, axes = plt.subplots(num_samples, 3, figsize=(15, 5 * num_samples))

    for i in range(num_samples):
        idx = np.random.randint(0, len(X))

        # Take the middle frame of the sequence
        middle_frame = X[idx][len(X[idx]) // 2]
        xoz_img = middle_frame[0]
        yoz_img = middle_frame[1]

        # Merge channels into an RGB image (for visualization only)
        rgb_img = np.stack(
            [xoz_img, yoz_img, np.zeros_like(xoz_img)],
            axis=2
        )

        axes[i, 0].imshow(xoz_img, cmap='gray')
        axes[i, 0].set_title(
            f"Sample {idx}: XOZ projection (Class: {class_names[y[idx]]})"
        )
        axes[i, 0].axis('off')

        axes[i, 1].imshow(yoz_img, cmap='gray')
        axes[i, 1].set_title(f"Sample {idx}: YOZ projection")
        axes[i, 1].axis('off')

        axes[i, 2].imshow(rgb_img)
        axes[i, 2].set_title(f"Sample {idx}: Combined view")
        axes[i, 2].axis('off')

    plt.tight_layout()
    plt.savefig("D:/Ti/Py_mmWave_Roformer/sample_visualization.png")
    plt.show()


# Visualize samples
visualize_samples(X, y, class_names, num_samples=5)


def normalize_and_save_data(X, y, save_path):
    """
    Normalize image data and save it as NumPy files.
    """
    os.makedirs(save_path, exist_ok=True)

    # Normalize to [0, 1]
    X_normalized = X.astype(np.float32) / 255.0

    np.save(os.path.join(save_path, "X.npy"), X_normalized)
    np.save(os.path.join(save_path, "y.npy"), y)

    # Save class names
    with open(os.path.join(save_path, "class_names.txt"), 'w') as f:
        for name in class_names:
            f.write(f"{name}\n")

    print(f"Data saved to: {save_path}")
    print(f"X shape: {X_normalized.shape}, y shape: {y.shape}")


# Normalize and save data
save_path = "D:/Ti/Py_mmWave_Roformer/processed_data"
normalize_and_save_data(X, y, save_path)


class ImageSequenceDataset(Dataset):
    """
    PyTorch dataset for image sequence classification.
    """

    def __init__(self, X_path, y_path):
        self.X = np.load(X_path)
        self.y = np.load(y_path)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        sequence = torch.FloatTensor(self.X[idx])
        label = torch.tensor(self.y[idx], dtype=torch.long)
        return sequence, label


# Create dataset instance
dataset = ImageSequenceDataset(
    os.path.join(save_path, "X.npy"),
    os.path.join(save_path, "y.npy")
)

# Split into training and test sets
indices = list(range(len(dataset)))
train_indices, test_indices = train_test_split(
    indices,
    test_size=0.2,
    random_state=42,
    stratify=dataset.y
)

from torch.utils.data import Subset

train_dataset = Subset(dataset, train_indices)
test_dataset = Subset(dataset, test_indices)

# Create data loaders
batch_size = 16
train_dataloader = DataLoader(
    train_dataset,
    batch_size=batch_size,
    shuffle=True,
    num_workers=2
)
test_dataloader = DataLoader(
    test_dataset,
    batch_size=batch_size,
    shuffle=False,
    num_workers=2
)

print(f"Training set size: {len(train_dataset)}")
print(f"Test set size: {len(test_dataset)}")
