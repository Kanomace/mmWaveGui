import pandas as pd
import matplotlib.pyplot as plt

# 读取xlsx文件中的点云数据
xlsx_file = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\03_22_2024_17_36_31\pHistBytes_1.xlsx'
df = pd.read_excel(xlsx_file)

# 提取X、Y和Z坐标列
x = df['X']
y = df['Y']
z = df['Z']

# 绘制点云图像
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()