import os

def rename_images(folder_path, start_time=28):
    # Get all image files in the folder (sorted by name)
    images = sorted(os.listdir(folder_path))

    # Rename images sequentially
    for i, filename in enumerate(images):
        if i>(shift - 1):
            print(i)
            # Extract the file extension (e.g., .jpg, .png)
            file_ext = os.path.splitext(filename)[1]

            # Create the new filename (e.g., t28.jpg, t29.png, etc.)
            new_name = images[i-shift]

            # Full paths for renaming
            old_path = os.path.join(folder_path, filename)
            new_path = os.path.join(folder_path, new_name)

            # Rename the file
            os.rename(old_path, new_path)
            print(f"Renamed {filename} -> {new_name}")

shift = 29
# Specify the folder containing the images
folder_path = "/home/eventcamera/data/dataset/dataset_23_jan/speed/rgb"  # Change this to your actual folder path
rename_images(folder_path)