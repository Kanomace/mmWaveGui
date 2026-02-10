import numpy as np
import tensorflow as tf

# Load data
def load_data(file_paths):
    data = []
    for file_path in file_paths:
        frame_data = np.load(file_path)
        data.append(frame_data)
    return np.array(data)

# Load the trained model
model_path = (
    "C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/"
    "tools/visualizers/Industrial_Visualizer/binData/trainData/"
    "saved_model/model.keras"
)
model = tf.keras.models.load_model(model_path)

# Data file paths
file_paths = [
    "C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/tools/visualizers/Industrial_Visualizer/binData/trainData/saved_model/data/pHistBytes_clustered_voxel_64.npy",
    "C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/tools/visualizers/Industrial_Visualizer/binData/trainData/saved_model/data/pHistBytes_clustered_voxel_65.npy",
    "C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/tools/visualizers/Industrial_Visualizer/binData/trainData/saved_model/data/pHistBytes_clustered_voxel_66.npy"
]

# Load data
window_size = 3
data = load_data(file_paths)

# Create sliding windows
data_windows = []
for i in range(len(data) - window_size + 1):
    window = data[i:i + window_size]
    data_windows.append(window)
data_windows = np.array(data_windows)

# Predict human pose types
predictions = model.predict(data_windows)
pose_types = np.argmax(predictions, axis=1)

# Print results
for i, pose_type in enumerate(pose_types):
    print(f"Window {i + 1} - Pose Type: {pose_type}")
