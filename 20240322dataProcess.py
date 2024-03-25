# @file        real_data
# @type        python
# @author      Kano
# @date        2024-03-18
#@brief        用于利用串口读取毫米波雷达的数据
from gui_parser import *
import pandas as pd
import os


# 定义输入文件和输出文件路径
input_file = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\03_25_2024_16_11_11\pHistBytes_15.bin'
# 分离输入文件的文件名和文件后缀
input_filename, input_ext = os.path.splitext(os.path.basename(input_file))
input_dir = os.path.dirname(input_file)

# 构造输出文件的路径，与输入文件具有相同的文件名（除了文件后缀）
output_txt = os.path.join(input_dir, input_filename + '.txt')
output_xlsx = os.path.join(input_dir, input_filename + '.xlsx')

def parse_input_data(file_path):
    # 读取输入文件的数据
    with open(file_path, 'rb') as file:
        data = file.read()

    # 解析数据
    parsed_data = parseStandardFrame(data)

    return parsed_data


def write_output_data(file_path, parsed_data):
    # 写入输出文件
    with open(file_path, 'w') as file:
        # 将解析结果写入文件
        file.write(str(parsed_data))

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # 解析输入数据
    parsed_data = parse_input_data(input_file)

    point_cloud_data = parsed_data['pointCloud']

    # 创建一个DataFrame对象来存储点云数据
    df = pd.DataFrame(point_cloud_data, columns=['X', 'Y', 'Z', 'Doppler', 'SNR', 'Noise', 'Track index'])

    # # 将解析结果写入输出文件
    write_output_data(output_txt, parsed_data)
    # 将DataFrame保存为xlsx文件
    df.to_excel(output_xlsx, index=False)