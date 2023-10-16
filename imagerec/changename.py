import os

folder_path = 'W8Testimg/uploads'  # Specify the folder path where the images are located
starting_index = 11  # Specify the starting index for the X value
file_extension = '.jpg'  # Specify the file extension of the images

current_index = starting_index

# List all files in the folder
files = os.listdir(folder_path)

# Filter only the image files based on the file extension
image_files = [file for file in files if file.endswith(file_extension)]

# Rename the image files
for old_name in image_files:
    new_name = f'{1665419523}_{current_index}_C{file_extension}'
    old_path = os.path.join(folder_path, old_name)
    new_path = os.path.join(folder_path, new_name)
    os.rename(old_path, new_path)
    print(f'Renamed: {old_name} -> {new_name}')
    current_index += 1