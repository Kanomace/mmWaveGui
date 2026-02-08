import os
import shutil
from natsort import natsorted
import openpyxl

folder_path = r'D:\Ti\Py_mmWave_Roformer\Dataset\Stationary'
file_prefix = 'pHistBytes_'
max_files = 2000

existing_files = [file for file in os.listdir(folder_path) if file.startswith(file_prefix)]
existing_files = natsorted(existing_files)

xlsx_files = [file for file in existing_files if file.endswith('.xlsx')]

if len(xlsx_files) < max_files:
    current_count = len(xlsx_files) + 1
    while current_count <= max_files:
        new_file_name = file_prefix + str(current_count) + '.xlsx'
        new_file_path = os.path.join(folder_path, new_file_name)

        # Create a new workbook
        workbook = openpyxl.Workbook()
        sheet = workbook.active

        # Set the column headers
        column_headers = ['X', 'Y', 'Z', 'Doppler', 'SNR', 'Noise', 'Track index']
        sheet.append(column_headers)

        # Save the workbook
        workbook.save(new_file_path)

        print(f"Generated file: {new_file_name}")
        current_count += 1