import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN

# 读取 Excel 文件
file_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\10framsPerFile\pHistBytes_1.xlsx'
df = pd.read_excel(file_path)

# 提取需要的列作为特征向量
features = df[['X', 'Y', 'Z', 'Doppler', 'SNR', 'Noise']].values

# 创建 DBSCAN 聚类器对象
dbscan = DBSCAN(eps=5, min_samples=1)  # 根据实际情况调整 eps 和 min_samples 参数

# 进行聚类
labels = dbscan.fit_predict(features)

# 输出聚类结果
for i in range(max(labels)+1):
    cluster_points = features[labels == i]
    print(f"Cluster {i+1}: {list(cluster_points)}")
    # 可视化聚类结果
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 设置坐标轴范围
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_zlim(0, 10)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 绘制背景网格
    background = (5, 5, 5)  # 原点位置
    ax.grid(True)
    ax.set_xticks(range(11))
    ax.set_yticks(range(11))
    ax.set_zticks(range(11))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_zticklabels([])

    # 绘制聚类簇数据
    ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2])

    plt.show()

noise_points = features[labels == -1]
print(f"Noise: {list(noise_points)}")

# 打印聚类结果
print("聚类结果：")
print(labels)