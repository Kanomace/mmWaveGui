import numpy as np
import os
import matplotlib.pyplot as plt
import re

# 设置体素文件夹路径
voxel_folder = r'D:\Ti\Py_mmWave_Roformer\Dataset\stationary.test\pHistBytes_clustered_voxel'

# 创建保存路径文件夹
save_path_YOZ = os.path.join(voxel_folder, 'pHistBytes_clustered_voxel_YOZ')
save_path_XOY = os.path.join(voxel_folder, 'pHistBytes_clustered_voxel_XOY')
save_path_XOZ = os.path.join(voxel_folder, 'pHistBytes_clustered_voxel_XOZ')

os.makedirs(save_path_YOZ, exist_ok=True)
os.makedirs(save_path_XOY, exist_ok=True)
os.makedirs(save_path_XOZ, exist_ok=True)


# 获取所有.npy文件并按数字顺序排序
def extract_number(filename):
    # 从文件名中提取数字部分
    match = re.search(r'pHistBytes_(\d+)\.npy', filename)
    if match:
        return int(match.group(1))
    return 0


# 获取并排序文件
npy_files = [f for f in os.listdir(voxel_folder) if f.endswith('.npy')]
npy_files.sort(key=extract_number)  # 按数字顺序排序

print(f"Found {len(npy_files)} .npy files, processing in order...")

# 按顺序处理每个文件
for file_name in npy_files:
    try:
        # 读取体素文件
        file_path = os.path.join(voxel_folder, file_name)
        voxel_data = np.load(file_path)

        # 提取XOY平面
        xoyslice = np.max(voxel_data, axis=2)
        xoyslice = np.rot90(xoyslice)
        xoyslice = np.where(xoyslice > 0, 1, 0)  # 将体素区域设为白色，空白区域设为黑色

        # 提取XOZ平面
        xozslice = np.max(voxel_data, axis=1)
        xozslice = np.rot90(xozslice)
        xozslice = np.where(xozslice > 0, 1, 0)  # 将体素区域设为白色，空白区域设为黑色

        # 提取YOZ平面
        yozslice = np.max(voxel_data, axis=0)
        yozslice = np.rot90(yozslice)
        yozslice = np.where(yozslice > 0, 1, 0)  # 将体素区域设为白色，空白区域设为黑色

        # 保存为图像文件
        base_name = os.path.splitext(file_name)[0]
        save_file_name_YOZ = os.path.join(save_path_YOZ, base_name + '_YOZ.png')
        save_file_name_XOY = os.path.join(save_path_XOY, base_name + '_XOY.png')
        save_file_name_XOZ = os.path.join(save_path_XOZ, base_name + '_XOZ.png')

        plt.imsave(save_file_name_YOZ, yozslice, cmap='gray')
        plt.imsave(save_file_name_XOY, xoyslice, cmap='gray')
        plt.imsave(save_file_name_XOZ, xozslice, cmap='gray')

        # 显示处理进度
        frame_num = extract_number(file_name)
        print(f'Processed frame {frame_num}/{len(npy_files)}: {file_name}')

    except Exception as e:
        print(f"Error occurred for {file_name}: {e}")

print("All frames processed successfully!")