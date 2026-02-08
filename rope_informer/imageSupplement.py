import glob
import os
import re
from PIL import Image

base_folder = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\traindata\9sign'
subfolders = [str(i) for i in range(1, 51)]

for subfolder in subfolders:
    folder_path = os.path.join(base_folder, subfolder)
    subfolders = ['pHistBytes_clustered_voxel_XOY', 'pHistBytes_clustered_voxel_XOZ', 'pHistBytes_clustered_voxel_YOZ']

    for subfolder in subfolders:
        subfolder_path = os.path.join(folder_path, subfolder)
        image_files = sorted(glob.glob(os.path.join(subfolder_path, '*.png')), key=lambda x: [int(c) if c.isdigit() else c for c in re.split(r'(\d+)', x)])
        num_images = len(image_files)

        if num_images < 15:
            image_names = [os.path.splitext(os.path.basename(image))[0] for image in image_files]
            image_extension = os.path.splitext(image_files[0])[1]
            first_image = Image.open(image_files[0])
            image_size = first_image.size

            for i in range(num_images + 1, 16):
                new_image_name = re.sub(r'\d+', str(i), image_names[0])
                new_image_name = f"{new_image_name}{image_extension}"
                new_image_path = os.path.join(subfolder_path, new_image_name)
                new_image = Image.new('RGB', image_size, color='black')
                new_image.save(new_image_path)
                print(f"Added: {new_image_path}")

        if num_images > 15:
            images_to_delete = image_files[15:]
            for image_to_delete in images_to_delete:
                os.remove(image_to_delete)
                print(f"Deleted: {image_to_delete}")