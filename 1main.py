# @file        real_data
# @type        python
# @author      Kano
# @date        2024-03-18
#@brief        用于利用串口读取毫米波雷达的数据
from gui_parser import *
import pandas as pd
import numpy as np

# 定义输入文件和输出文件路径
input_file = 'pHistBytes_1.txt'
output_file = 'ProcessedData.xlsx'  # 修改输出文件为xlsx格式
def parse_input_data(file_path):
    # 读取输入文件的数据
    with open(file_path, 'r') as file:
        data = file.read()
    # 移除换行符，并将数据拆分为帧
    frames = data.split('\n')
    # 解析每个帧的数据
    parsed_data = []
    for frame in frames:
        if len(frame) > 0:
            # 将帧数据转换为字节流
            frame_data = bytes.fromhex(frame)
            # 调用解析函数解析帧数据
            parsed_frame = parseStandardFrame(frame_data)
            # 将解析结果添加到列表中
            parsed_data.append(parsed_frame)
    return parsed_data

def extract_point_cloud(parsed_data):
    # 提取点云数据
    point_cloud = []
    for frame in parsed_data:
        point_cloud.extend(frame['pointCloud'])
    return point_cloud

def write_output_data(file_path, point_cloud):
    # 创建DataFrame对象
    df = pd.DataFrame(point_cloud, columns=['X', 'Y', 'Z', 'Doppler', 'SNR', 'Noise', 'Track index'])
    # 将DataFrame写入Excel文件
    df.to_excel(file_path, index=False)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # 解析输入数据
    parsed_data = parse_input_data(input_file)

    # 提取点云数据
    point_cloud = extract_point_cloud(parsed_data)

    # 将解析结果写入输出文件
    write_output_data(output_file, point_cloud)