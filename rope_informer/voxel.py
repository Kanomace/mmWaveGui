import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 设置体素文件夹路径
folder_path = r'D:\Ti\Py_mmWave_Roformer\Dataset\Stationary\pHistBytes_clustered_voxel'

# 获取体素文件列表
voxel_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.npy')])

# 创建 3D 图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 创建动画更新函数
def update_plot(frame):
    try:
        # 读取体素文件数据
        voxel_file = os.path.join(folder_path, voxel_files[frame])
        voxel_grid = np.load(voxel_file)
        # 清除前一帧的绘图
        ax.clear()
        # 绘制体素
        ax.voxels(voxel_grid, facecolors='b', edgecolor='k')
        # 设置坐标轴标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # 设置坐标轴范围
        ax.set_xlim(0, voxel_grid.shape[0])
        ax.set_ylim(0, voxel_grid.shape[1])
        ax.set_zlim(0, voxel_grid.shape[2])
        # 设置图形标题
        ax.set_title(f'Frame {frame + 1}/{len(voxel_files)}')
    except Exception as e:
        print(f"Error occurred in frame {frame + 1}: {e}")

# 创建动画
animation = FuncAnimation(fig, update_plot, frames=len(voxel_files), interval=500)

# 显示动画
plt.show()