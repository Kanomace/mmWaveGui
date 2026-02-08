import os
import re
import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.distance import euclidean

"""
修正z轴上的欧氏距离
"""
z_scale_factor = 0.5  # 用于缩放z轴坐标的比例因子
esp_set = 1.3
min_samples_set = 10

def z_corrected_euclidean(u, v):
    z_distance = abs(u[2] - v[2]) * z_scale_factor
    xy_distance = euclidean(u[:2], v[:2])
    corrected_distance = np.sqrt(xy_distance ** 2 + z_distance ** 2)
    return corrected_distance

def natural_sort_key(s):
    """
    自然排序的键函数
    """
    return [int(c) if c.isdigit() else c.lower() for c in re.split('([0-9]+)', s)]

def cluster_point_cloud_folder(folder_path, file_prefix, output_path):
    # 获取文件夹中符合前缀的点云数据文件
    file_list = [f for f in os.listdir(folder_path) if f.startswith(file_prefix) and f.endswith('.xlsx')]
    file_list.sort(key=natural_sort_key)  # 使用自然排序对文件名进行排序

    for file_name in file_list:
        # 读取点云数据文件
        file_path = os.path.join(folder_path, file_name)
        df = pd.read_excel(file_path)

        # 提取数据列
        x = df['X']   # 对 X 坐标进行平移
        y = df['Y']   # 对 Y 坐标进行平移
        z = df['Z']   # 对 Z 坐标进行平移

        # 将点云数据转换为NumPy数组
        points = np.column_stack((x, y, z))

        # 执行DBSCAN聚类
        dbscan = DBSCAN(eps=esp_set, min_samples=min_samples_set, metric=z_corrected_euclidean)  # 使用自定义的距离度量函数
        labels = dbscan.fit_predict(points)

        # 获取聚类最多的簇的标签和数量
        unique_labels, counts = np.unique(labels, return_counts=True)
        max_cluster_label = unique_labels[np.argmax(counts)]

        # 提取聚类最多的簇的点云数据
        x_cluster = x[labels == max_cluster_label]
        y_cluster = y[labels == max_cluster_label]
        z_cluster = z[labels == max_cluster_label]

        # 生成处理后的簇的DataFrame
        clustered_df = pd.DataFrame({'X': x_cluster, 'Y': y_cluster, 'Z': z_cluster})

        # 保存处理后的簇为xlsx文件
        output_file_path = os.path.join(output_path, file_name.replace(file_prefix, file_prefix + 'clustered_'))
        clustered_df.to_excel(output_file_path, index=False)

# # 示例用法
# folder_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\3tumble\2'
# output_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\3tumble\2\pHistBytes_clustered'
# # 创建目标文件夹（如果不存在）
# if not os.path.exists(output_path):
#     os.makedirs(output_path)
#
# file_prefix = 'pHistBytes_'
# cluster_point_cloud_folder(folder_path, file_prefix,output_path)

base_folder = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\2stand'
file_prefix = 'pHistBytes_'

for i in range(1, 51):
    folder_path = os.path.join(base_folder, str(i))
    output_path = os.path.join(folder_path, 'pHistBytes_clustered')

    # 创建目标文件夹（如果不存在）
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    cluster_point_cloud_folder(folder_path, file_prefix, output_path)
    print(f'Saved DBSCAN Result for in folder {i}')