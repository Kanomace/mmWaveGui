import matplotlib

matplotlib.use('TkAgg')  # Use the TkAgg backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import os
import re
from tqdm import tqdm


def natural_sort_key(s):
    """
    Natural sorting key function to ensure numeric ordering in filenames.
    """
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('(\d+)', s)]


def load_and_animate_voxels(folder_path, max_frames=100):
    """
    Load voxel .npy files from a folder and display them as a 3D animation.

    Args:
        folder_path (str): Path to the folder containing voxel .npy files.
        max_frames (int): Maximum number of frames to animate.
    """

    npy_files = [f for f in os.listdir(folder_path) if f.endswith('.npy')]
    npy_files.sort(key=natural_sort_key)

    if not npy_files:
        print(f"No .npy files found in folder {folder_path}")
        return

    # Limit the number of frames
    if len(npy_files) > max_frames:
        npy_files = npy_files[:max_frames]

    print(f"Loaded {len(npy_files)} files (natural order):")
    for i, f in enumerate(npy_files[:10]):
        print(f"  {i + 1}: {f}")
    if len(npy_files) > 10:
        print("  ...")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Set initial view angle
    ax.view_init(elev=30, azim=45)

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Load the first frame to set axis limits
    first_data = np.load(os.path.join(folder_path, npy_files[0]))
    ax.set_xlim(0, first_data.shape[0])
    ax.set_ylim(0, first_data.shape[1])
    ax.set_zlim(0, first_data.shape[2])

    print("Preloading data...")
    all_data = []
    for file in tqdm(npy_files, desc="Loading data"):
        data = np.load(os.path.join(folder_path, file))
        all_data.append(data)

    def update(frame):
        ax.clear()
        data = all_data[frame]

        # Get coordinates of occupied voxels
        x, y, z = np.indices(data.shape)
        x, y, z = x[data > 0], y[data > 0], z[data > 0]

        scatter = ax.scatter(
            x, y, z,
            c='red', marker='o', s=20, alpha=0.6
        )

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(0, first_data.shape[0])
        ax.set_ylim(0, first_data.shape[1])
        ax.set_zlim(0, first_data.shape[2])

        ax.view_init(elev=30, azim=45)

        ax.set_title(
            f"Frame: {frame + 1}/{len(npy_files)} - {npy_files[frame]}"
        )

        return scatter,

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(npy_files),
        interval=100,
        blit=False,
        repeat=True
    )

    plt.tight_layout()
    plt.show()


folder_path = r'D:\Ti\Py_mmWave_Roformer\Dataset\huangzhouyang.walk\pHistBytes_clustered_voxel'
load_and_animate_voxels(folder_path, max_frames=5000)
