import os
import re
import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.distance import euclidean

"""
Corrected Euclidean distance along the z-axis
"""
z_scale_factor = 0.5  # Scaling factor applied to the z-axis
esp_set = 1.3
min_samples_set = 10


def z_corrected_euclidean(u, v):
    z_distance = abs(u[2] - v[2]) * z_scale_factor
    xy_distance = euclidean(u[:2], v[:2])
    corrected_distance = np.sqrt(xy_distance ** 2 + z_distance ** 2)
    return corrected_distance


def natural_sort_key(s):
    """
    Natural sorting key function
    """
    return [int(c) if c.isdigit() else c.lower()
            for c in re.split('([0-9]+)', s)]


def cluster_point_cloud_folder(folder_path, file_prefix, output_path):
    # Get point cloud Excel files in the folder that match the given prefix
    file_list = [
        f for f in os.listdir(folder_path)
        if f.startswith(file_prefix) and f.endswith('.xlsx')
    ]
    file_list.sort(key=natural_sort_key)  # Sort filenames using natural order

    for file_name in file_list:
        # Load point cloud data from Excel file
        file_path = os.path.join(folder_path, file_name)
        df = pd.read_excel(file_path)

        # Extract coordinate columns
        x = df['X']   # X coordinate
        y = df['Y']   # Y coordinate
        z = df['Z']   # Z coordinate

        # Convert point cloud data to a NumPy array
        points = np.column_stack((x, y, z))

        # Perform DBSCAN clustering
        dbscan = DBSCAN(
            eps=esp_set,
            min_samples=min_samples_set,
            metric=z_corrected_euclidean  # Use the custom distance metric
        )
        labels = dbscan.fit_predict(points)

        # Identify the label and size of the largest cluster
        unique_labels, counts = np.unique(labels, return_counts=True)
        max_cluster_label = unique_labels[np.argmax(counts)]

        # Extract point cloud data of the largest cluster
        x_cluster = x[labels == max_cluster_label]
        y_cluster = y[labels == max_cluster_label]
        z_cluster = z[labels == max_cluster_label]

        # Create a DataFrame for the clustered point cloud
        clustered_df = pd.DataFrame({
            'X': x_cluster,
            'Y': y_cluster,
            'Z': z_cluster
        })

        # Save the clustered result as an Excel file
        output_file_path = os.path.join(
            output_path,
            file_name.replace(file_prefix, file_prefix + 'clustered_')
        )
        clustered_df.to_excel(output_file_path, index=False)


# # Example usage
# folder_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\3tumble\2'
# output_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\3tumble\2\pHistBytes_clustered'
# # Create output folder if it does not exist
# if not os.path.exists(output_path):
#     os.makedirs(output_path)
#
# file_prefix = 'pHistBytes_'
# cluster_point_cloud_folder(folder_path, file_prefix, output_path)


base_folder = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\2stand'
file_prefix = 'pHistBytes_'

for i in range(1, 51):
    folder_path = os.path.join(base_folder, str(i))
    output_path = os.path.join(folder_path, 'pHistBytes_clustered')

    # Create output folder if it does not exist
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    cluster_point_cloud_folder(folder_path, file_prefix, output_path)
    print(f'Saved DBSCAN results for folder {i}')
