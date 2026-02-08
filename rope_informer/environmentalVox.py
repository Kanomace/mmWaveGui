import os
import pandas as pd
import numpy as np
from tqdm import tqdm
import json
import argparse
import re


def natural_sort_key(s):
    """
    自然排序键函数，确保数字按数值大小排序
    例如: ['file1', 'file2', 'file10'] 而不是 ['file1', 'file10', 'file2']
    """
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('(\d+)', s)]


def voxelization(data, grid_size, boundaries):
    """
    将点云数据转换为体素网格

    参数:
    data: 包含x,y,z坐标的DataFrame
    grid_size: 体素网格尺寸 (x, y, z)
    boundaries: 点云边界范围字典

    返回:
    voxel_grid: 体素网格数组
    """
    # 处理空点云情况
    if data.empty:
        return np.zeros(grid_size, dtype=np.uint8)

    # 计算每个体素的大小
    voxel_size = (
        (boundaries['x'][1] - boundaries['x'][0]) / grid_size[0],
        (boundaries['y'][1] - boundaries['y'][0]) / grid_size[1],
        (boundaries['z'][1] - boundaries['z'][0]) / grid_size[2]
    )

    # 计算每个点所属的体素坐标，并限制在网格范围内
    x_indices = ((data['x'] - boundaries['x'][0]) / voxel_size[0]).astype(int)
    y_indices = ((data['y'] - boundaries['y'][0]) / voxel_size[1]).astype(int)
    z_indices = ((data['z'] - boundaries['z'][0]) / voxel_size[2]).astype(int)

    # 限制索引在有效范围内
    x_indices = np.clip(x_indices, 0, grid_size[0] - 1)
    y_indices = np.clip(y_indices, 0, grid_size[1] - 1)
    z_indices = np.clip(z_indices, 0, grid_size[2] - 1)

    # 创建体素网格
    voxel_grid = np.zeros(grid_size, dtype=np.uint8)

    # 将点云数据填充到体素网格中
    voxel_grid[x_indices, y_indices, z_indices] = 1

    return voxel_grid


def process_excel_files(input_folder, output_folder, grid_size, boundaries, skip_existing=True):
    """
    处理指定文件夹中的所有Excel文件

    参数:
    input_folder: 包含Excel文件的文件夹路径
    output_folder: 输出文件夹路径
    grid_size: 体素网格尺寸
    boundaries: 点云边界范围
    skip_existing: 是否跳过已处理的文件

    返回:
    处理成功的文件数量
    """
    # 创建保存体素文件的文件夹
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 获取文件夹中的Excel文件列表并按自然顺序排序
    excel_files = [f for f in os.listdir(input_folder) if f.endswith('.xlsx')]
    excel_files.sort(key=natural_sort_key)  # 使用自然排序

    print(f"找到 {len(excel_files)} 个Excel文件，按自然排序:")
    for i, f in enumerate(excel_files[:10]):  # 显示前10个文件名
        print(f"  {i + 1}: {f}")
    if len(excel_files) > 10:
        print("  ...")

    processed_count = 0
    error_count = 0

    # 使用进度条显示处理进度
    for frame, excel_file in enumerate(tqdm(excel_files, desc="Processing Excel files")):
        try:
            # 检查输出文件是否已存在
            output_file = os.path.join(output_folder, f'{os.path.splitext(excel_file)[0]}.npy')
            if skip_existing and os.path.exists(output_file):
                processed_count += 1
                continue

            # 读取 Excel 文件数据
            df = pd.read_excel(os.path.join(input_folder, excel_file))

            # 检查必要的列是否存在
            if not all(col in df.columns for col in ['X', 'Y', 'Z']):
                print(f"Warning: File {excel_file} doesn't contain required columns (X, Y, Z)")
                error_count += 1
                continue

            # 提取 xyz 列数据
            x = df['X']
            y = df['Y']
            z = df['Z']

            # 创建点云数据 DataFrame
            point_cloud = pd.DataFrame({'x': x, 'y': y, 'z': z})

            # 筛选符合边界范围的点云数据
            point_cloud = point_cloud[
                (point_cloud['x'] >= boundaries['x'][0]) & (point_cloud['x'] <= boundaries['x'][1]) &
                (point_cloud['y'] >= boundaries['y'][0]) & (point_cloud['y'] <= boundaries['y'][1]) &
                (point_cloud['z'] >= boundaries['z'][0]) & (point_cloud['z'] <= boundaries['z'][1])
                ]

            # 进行体素化
            voxel_grid = voxelization(point_cloud, grid_size, boundaries)

            # 保存体素文件
            np.save(output_file, voxel_grid)
            processed_count += 1

        except Exception as e:
            error_count += 1
            print(f"Error occurred in file {excel_file}: {e}")

    return processed_count, error_count


def main():
    # 设置文件夹路径
    base_folder = r'D:\Ti\Py_mmWave_Roformer\Dataset\stationary.test'
    output_folder = os.path.join(base_folder, 'pHistBytes_clustered_voxel')

    # 定义体素网格大小
    grid_size = (25, 15, 25)  # (x, y, z)

    # 定义点云边界
    boundaries = {
        'x': (-5, 5),  # X轴范围为 -5 到 5 米
        'y': (0, 6),  # Y轴范围为 0 到 6 米
        'z': (-5, 5)  # Z轴范围为 -5 到 5 米
    }

    print(f"Input folder: {base_folder}")
    print(f"Output folder: {output_folder}")
    print(f"Grid size: {grid_size}")
    print(f"Boundaries: {boundaries}")

    # 处理所有Excel文件
    processed, errors = process_excel_files(base_folder, output_folder, grid_size, boundaries)

    print(f"\nProcessing completed!")
    print(f"Processed files: {processed}")
    print(f"Errors: {errors}")


if __name__ == '__main__':
    main()