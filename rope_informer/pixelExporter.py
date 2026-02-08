import os
from PIL import Image
from openpyxl import Workbook

# 文件夹路径
base_folder_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\9sign'

# 文件夹数量
folder_count = 50

# 文件夹后缀列表
folder_suffixes = ['pHistBytes_clustered_voxel_XOY', 'pHistBytes_clustered_voxel_XOZ', 'pHistBytes_clustered_voxel_YOZ']

# 遍历文件夹
for i in range(1, folder_count + 1):
    folder_path = os.path.join(base_folder_path, str(i))

    # 遍历文件夹后缀
    for suffix in folder_suffixes:
        # 文件夹完整路径
        full_folder_path = os.path.join(folder_path, suffix)

        # 获取文件夹中的图像文件列表
        image_files = [f for f in os.listdir(full_folder_path) if f.endswith('.png')]
        # 图像文件排序
        image_files.sort(key=lambda x: int(x.split('_')[-2]))

        # 创建Excel工作簿
        workbook = Workbook()
        sheet = workbook.active

        # 创建表头
        header = []
        pixel_values_combined = []

        # 遍历图像文件并将数据写入表头和像素值列表
        for j, image_file in enumerate(image_files):
            image_path = os.path.join(full_folder_path, image_file)
            image = Image.open(image_path).convert('L')  # 打开并转换为灰度图像
            pixel_values = list(image.getdata())

            # 在第一个图像时调整表头为图像的像素数量
            if j == 0:
                header = ['pixel' + str(k + 1) for k in range(len(pixel_values))]

            # 将当前图像的像素值添加到像素值列表
            pixel_values_combined.extend(pixel_values)

        # 将表头和像素值列表写入Excel文件
        sheet.append(header)
        sheet.append(pixel_values_combined)

        # 保存Excel文件
        excel_file_path = os.path.join(full_folder_path, 'image_data.xlsx')
        workbook.save(excel_file_path)

        print('Excel文件已保存：', excel_file_path)