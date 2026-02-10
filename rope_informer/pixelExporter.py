import os
from PIL import Image
from openpyxl import Workbook

# Base folder path
base_folder_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\9sign'

# Number of subfolders
folder_count = 50

# List of subfolder suffixes
folder_suffixes = [
    'pHistBytes_clustered_voxel_XOY',
    'pHistBytes_clustered_voxel_XOZ',
    'pHistBytes_clustered_voxel_YOZ'
]

# Iterate through folders
for i in range(1, folder_count + 1):
    folder_path = os.path.join(base_folder_path, str(i))

    # Iterate through subfolder suffixes
    for suffix in folder_suffixes:
        # Full subfolder path
        full_folder_path = os.path.join(folder_path, suffix)

        # Get list of image files in the folder
        image_files = [f for f in os.listdir(full_folder_path) if f.endswith('.png')]
        # Sort image files
        image_files.sort(key=lambda x: int(x.split('_')[-2]))

        # Create an Excel workbook
        workbook = Workbook()
        sheet = workbook.active

        # Initialize header and pixel value list
        header = []
        pixel_values_combined = []

        # Iterate through image files and write data to header and pixel list
        for j, image_file in enumerate(image_files):
            image_path = os.path.join(full_folder_path, image_file)
            image = Image.open(image_path).convert('L')  # Open and convert image to grayscale
            pixel_values = list(image.getdata())

            # Initialize header based on the number of pixels in the first image
            if j == 0:
                header = ['pixel' + str(k + 1) for k in range(len(pixel_values))]

            # Append pixel values of the current image
            pixel_values_combined.extend(pixel_values)

        # Write header and pixel values to the Excel file
        sheet.append(header)
        sheet.append(pixel_values_combined)

        # Save the Excel file
        excel_file_path = os.path.join(full_folder_path, 'image_data.xlsx')
        workbook.save(excel_file_path)

        print('Excel file has been saved:', excel_file_path)
