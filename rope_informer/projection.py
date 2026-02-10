import numpy as np
import os
import matplotlib.pyplot as plt
import re

# Set the voxel folder path
voxel_folder = r'D:\Ti\Py_mmWave_Roformer\Dataset\stationary.test\pHistBytes_clustered_voxel'

# Create output directories
save_path_YOZ = os.path.join(voxel_folder, 'pHistBytes_clustered_voxel_YOZ')
save_path_XOY = os.path.join(voxel_folder, 'pHistBytes_clustered_voxel_XOY')
save_path_XOZ = os.path.join(voxel_folder, 'pHistBytes_clustered_voxel_XOZ')

os.makedirs(save_path_YOZ, exist_ok=True)
os.makedirs(save_path_XOY, exist_ok=True)
os.makedirs(save_path_XOZ, exist_ok=True)


# Extract numeric index from filename
def extract_number(filename):
    # Extract the numeric part from the filename
    match = re.search(r'pHistBytes_(\d+)\.npy', filename)
    if match:
        return int(match.group(1))
    return 0


# Get all .npy files and sort them numerically
npy_files = [f for f in os.listdir(voxel_folder) if f.endswith('.npy')]
npy_files.sort(key=extract_number)

print(f"Found {len(npy_files)} .npy files, processing in order...")

# Process each file sequentially
for file_name in npy_files:
    try:
        # Load voxel file
        file_path = os.path.join(voxel_folder, file_name)
        voxel_data = np.load(file_path)

        # Extract the XOY plane
        xoyslice = np.max(voxel_data, axis=2)
        xoyslice = np.rot90(xoyslice)
        xoyslice = np.where(xoyslice > 0, 1, 0)  # Set occupied voxels to white, empty space to black

        # Extract the XOZ plane
        xozslice = np.max(voxel_data, axis=1)
        xozslice = np.rot90(xozslice)
        xozslice = np.where(xozslice > 0, 1, 0)  # Set occupied voxels to white, empty space to black

        # Extract the YOZ plane
        yozslice = np.max(voxel_data, axis=0)
        yozslice = np.rot90(yozslice)
        yozslice = np.where(yozslice > 0, 1, 0)  # Set occupied voxels to white, empty space to black

        # Save slices as image files
        base_name = os.path.splitext(file_name)[0]
        save_file_name_YOZ = os.path.join(save_path_YOZ, base_name + '_YOZ.png')
        save_file_name_XOY = os.path.join(save_path_XOY, base_name + '_XOY.png')
        save_file_name_XOZ = os.path.join(save_path_XOZ, base_name + '_XOZ.png')

        plt.imsave(save_file_name_YOZ, yozslice, cmap='gray')
        plt.imsave(save_file_name_XOY, xoyslice, cmap='gray')
        plt.imsave(save_file_name_XOZ, xozslice, cmap='gray')

        # Display processing progress
        frame_num = extract_number(file_name)
        print(f'Processed frame {frame_num}/{len(npy_files)}: {file_name}')

    except Exception as e:
        print(f"Error occurred for {file_name}: {e}")

print("All frames processed successfully!")
