import os

folder_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\9sign'  # Replace with your folder path

# Get all subfolders in the directory
subfolders = [
    f for f in os.listdir(folder_path)
    if os.path.isdir(os.path.join(folder_path, f))
]

# Sort subfolders by name
sorted_subfolders = sorted(subfolders)

# Rename each subfolder
for i, subfolder in enumerate(sorted_subfolders):
    old_path = os.path.join(folder_path, subfolder)
    new_name = str(i + 1)
    new_path = os.path.join(folder_path, new_name)
    os.rename(old_path, new_path)
