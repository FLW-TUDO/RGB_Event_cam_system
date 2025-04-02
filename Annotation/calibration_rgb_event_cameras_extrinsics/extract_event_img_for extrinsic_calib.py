import os
import json
import shutil
import re

# Paths
json_file_path = "/media/eventcamera/event_data/calibration/Extrinsic/fourth_calib/vicon_coordinates.json"  # Update with your JSON file path
source_root = "/media/eventcamera/event_data/calibration/Extrinsic/third_calib/e2calib_25/"  # Update with the root containing the 50 folders
destination_folder = "/media/eventcamera/event_data/calibration/Extrinsic/fourth_calib/images"  # Folder to save selected images

# Ensure destination folder exists
os.makedirs(destination_folder, exist_ok=True)

# Load timestamps from JSON
with open(json_file_path, "r") as f:
    vicon_data = json.load(f)  # Assuming it's a list of 50 timestamps
# lebgth of vicom_data
length = len(vicon_data)

def find_closest_elements(A, B):
    closest_b = min(B, key=lambda x: abs(x - A))
    result = closest_b
    B.remove(closest_b)
    print(A,closest_b)
    return result

event_files = [f for f in os.listdir(source_root) if f.endswith('.png')]
event_timestamps = sorted([int(f[:-4]) for f in event_files])  # Extract numbers & sort

# Process each timestamp from vicon_data
for i in range(length):
    timestamp_vicon = int(vicon_data[str(i)]["timestamp"] * 1e9)  # Convert to nanoseconds

    # Find closest timestamp
    closest_timestamp = find_closest_elements(timestamp_vicon, event_timestamps)

    # Build source and destination paths
    src_path = os.path.join(source_root, f"{closest_timestamp}.png")
    dest_path = os.path.join(destination_folder, f"{i}.png")  # Rename to avoid conflicts

    # Copy the file
    shutil.copy(src_path, dest_path)
    #print(f"Copied {src_path} -> {dest_path}")
'''
# Iterate over 50 folders
for i in range(0, 51):
    folder_path = os.path.join(source_root, f"{i}",'e2calib')  # Adjust folder naming as needed
    timestamp = vicon_data[str(i)]["timestamp"]
    # delete decimal point from timestamp. Keep the integer part and also the decimal part as a single integer
    timestamp = int(timestamp * 10000000)

    if not os.path.exists(folder_path):
        print(f"Skipping {folder_path}, folder not found.")
        continue

    image_files = [f for f in os.listdir(folder_path) if f.lower().endswith((".png", ".jpg", ".jpeg"))]

    if not image_files:
        print(f"No images found in {folder_path}")
        continue

    # Find the image with the closest timestamp
    closest_image = min(image_files, key=lambda img: abs(extract_timestamp(img) - timestamp))

    # Copy the closest-matching image to destination folder
    src_path = os.path.join(folder_path, closest_image)
    dest_path = os.path.join(destination_folder, f"{i}.png")  # Rename to avoid conflicts
    shutil.copy(src_path, dest_path)

    print(f"Copied {closest_image} from {folder_path} to {destination_folder}")
'''
print("✅ Image selection completed!")

