import os

folder_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\9sign'  # 替换为你的文件夹路径

# 获取文件夹中的所有子文件夹
subfolders = [f for f in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, f))]

# 按名称排序子文件夹
sorted_subfolders = sorted(subfolders)

# 为每个子文件夹重命名
for i, subfolder in enumerate(sorted_subfolders):
    old_path = os.path.join(folder_path, subfolder)
    new_name = str(i + 1)
    new_path = os.path.join(folder_path, new_name)
    os.rename(old_path, new_path)