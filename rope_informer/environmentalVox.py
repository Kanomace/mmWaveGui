import os
import pandas as pd
import numpy as np
from tqdm import tqdm
import json
import argparse
import re


def natural_sort_key(s):
    """
    Natural sorting key function to ensure numbers are sorted
    by their numerical value.

    Example:
        ['file1', 'file2', 'file10'] instead of ['file1', 'file10', 'file2']
    """
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('(\d+)', s)]


def voxelization(data, grid_size, boundaries):
    """
    Convert point cloud data into a voxel grid.

    Args:
        data (DataFrame):
            DataFrame containing x, y, z coordinates.
        grid_size (tuple):
            Voxel grid size (x, y, z).
        boundaries (dict):
            Dictionary specifying the point cloud boundaries.

    Returns:
        voxel_grid (np.ndarray):
            Generated voxel grid array.
    """
    # Handle empty point cloud
    if data.empty:
        return np.zeros(grid_size, dtype=np.uint8)

    # Compute voxel size along each dimension
    voxel_size = (
        (boundaries['x'][1] - boundaries['x'][0]) / grid_size[0],
        (boundaries['y'][1] - boundaries['y'][0]) / grid_size[1],
        (boundaries['z'][1] - boundaries['z'][0]) / grid_size[2]
    )

    # Compute voxel indices for each point and clamp them to valid range
    x_indices = ((data['x'] - boundaries['x'][0]) / voxel_size[0]).astype(int)
    y_indices = ((data['y'] - boundaries['y'][0]) / voxel_size[1]).astype(int)
    z_indices = ((data['z'] - boundaries['z'][0]) / voxel_size[2]).astype(int)

    x_indices = np.clip(x_indices, 0, grid_size[0] - 1)
    y_indices = np.clip(y_indices, 0, grid_size[1] - 1)
    z_indices = np.clip(z_indices, 0, grid_size[2] - 1)

    # Initialize voxel grid
    voxel_grid = np.zeros(grid_size, dtype=np.uint8)

    # Fill voxel grid with point cloud occupancy
    voxel_grid[x_indices, y_indices, z_indices] = 1

    return voxel_grid


def process_excel_files(input_folder, output_folder, grid_size, boundaries, skip_existing=True):
    """
    Process all Excel files in the specified folder.

    Args:
        input_folder (str):
            Path to the folder containing Excel files.
        output_folder (str):
            Path to the output folder.
        grid_size (tuple):
            Voxel grid size.
        boundaries (dict):
            Point cloud boundary ranges.
        skip_existing (bool):
            Whether to skip already processed files.

    Returns:
        processed_count (int):
            Number of successfully processed files.
        error_count (int):
            Number of files that failed during processing.
    """
    # Create output folder if it does not exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Get Excel files and sort them using natural order
    excel_files = [f for f in os.listdir(input_folder) if f.endswith('.xlsx')]
    excel_files.sort(key=natural_sort_key)

    print(f"Found {len(excel_files)} Excel files (natural order):")
    for i, f in enumerate(excel_files[:10]):
        print(f"  {i + 1}: {f}")
    if len(excel_files) > 10:
        print("  ...")

    processed_count = 0
    error_count = 0

    # Process files with a progress bar
    for frame, excel_file in enumerate(tqdm(excel_files, desc="Processing Excel files")):
        try:
            output_file = os.path.join(
                output_folder, f'{os.path.splitext(excel_file)[0]}.npy'
            )

            # Skip if output already exists
            if skip_existing and os.path.exists(output_file):
                processed_count += 1
                continue

            # Load Excel data
            df = pd.read_excel(os.path.join(input_folder, excel_file))

            # Check required columns
            if not all(col in df.columns for col in ['X', 'Y', 'Z']):
                print(
                    f"Warning: File {excel_file} does not contain "
                    f"required columns (X, Y, Z)"
                )
                error_count += 1
                continue

            # Extract XYZ data
            x = df['X']
            y = df['Y']
            z = df['Z']

            # Create point cloud DataFrame
            point_cloud = pd.DataFrame({'x': x, 'y': y, 'z': z})

            # Filter points within specified boundaries
            point_cloud = point_cloud[
                (point_cloud['x'] >= boundaries['x'][0]) & (point_cloud['x'] <= boundaries['x'][1]) &
                (point_cloud['y'] >= boundaries['y'][0]) & (point_cloud['y'] <= boundaries['y'][1]) &
                (point_cloud['z'] >= boundaries['z'][0]) & (point_cloud['z'] <= boundaries['z'][1])
            ]

            # Perform voxelization
            voxel_grid = voxelization(point_cloud, grid_size, boundaries)

            # Save voxel grid
            np.save(output_file, voxel_grid)
            processed_count += 1

        except Exception as e:
            error_count += 1
            print(f"Error occurred while processing file {excel_file}: {e}")

    return processed_count, error_count


def main():
    # Set folder paths
    base_folder = r'D:\Ti\Py_mmWave_Roformer\Dataset\stationary.test'
    output_folder = os.path.join(base_folder, 'pHistBytes_clustered_voxel')

    # Define voxel grid size
    grid_size = (25, 15, 25)  # (x, y, z)

    # Define point cloud boundaries
    boundaries = {
        'x': (-5, 5),   # X-axis range: -5 to 5 meters
        'y': (0, 6),    # Y-axis range: 0 to 6 meters
        'z': (-5, 5)    # Z-axis range: -5 to 5 meters
    }

    print(f"Input folder: {base_folder}")
    print(f"Output folder: {output_folder}")
    print(f"Grid size: {grid_size}")
    print(f"Boundaries: {boundaries}")

    # Process all Excel files
    processed, errors = process_excel_files(
        base_folder, output_folder, grid_size, boundaries
    )

    print("\nProcessing completed!")
    print(f"Processed files: {processed}")
    print(f"Errors: {errors}")


if __name__ == '__main__':
    main()
