import binascii

file_path = r"C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\03_25_2024_16_00_30\pHistBytes_1.bin"
output_file_path = r"C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\03_25_2024_16_00_30\pHistBytes_1.txt"

with open(file_path, "rb") as file:
    data = file.read()
    hex_data = binascii.hexlify(data).decode()

with open(output_file_path, "w") as output_file:
    output_file.write(hex_data)

print("文件已保存为txt格式。")