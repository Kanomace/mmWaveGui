# @file        real_data
# @type        python
# @author      Kano
# @date        2024-03-18
#@brief        用于利用串口读取毫米波雷达的数据
from gui_parser import *

# 定义输入文件和输出文件路径
input_file = 'originData.txt'
output_file = 'ProcessedData.txt'

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

def write_output_data(file_path, parsed_data):
    # 写入输出文件
    with open(file_path, 'w') as file:
        for frame_data in parsed_data:
            # 将解析结果写入文件
            file.write(str(frame_data) + '\n')

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # 解析输入数据
    parsed_data = parse_input_data(input_file)

    # 将解析结果写入输出文件
    write_output_data(output_file, parsed_data)
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
