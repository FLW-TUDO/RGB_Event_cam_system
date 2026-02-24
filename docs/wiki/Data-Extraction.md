# Data Extraction

This page describes the standard data extraction workflow used in this repo.

## 1) Inputs

Typical inputs are ROS bags that contain:

- `/dvxplorer_left/events`
- `/dvxplorer_right/events`
- `/camera/image_raw` (RGB)
- Vicon topics (event camera system + object) if available

## 2) Main Extractor Script

The main extractor is:

- `extract_rgb_events_vicon_data_from_bag.py`

It reads a scene list from `scene_data.json` under the configured data root (example: `/mnt/smbshare/`). It creates folders and writes:

- RGB images (`.jpg`)
- Vicon transformation JSON files
- Optional event `.npy` dumps (commented out in the script)

Before running, set the correct `root` path in the script.

## 3) RGB Extraction Only

RGB extraction is also available via:

- `calibration_rgb_event_cameras_extrinsics/extract_rgb_img_from_bag.py`

This is useful for calibration or sanity checks.

## 4) Output Locations

The extractor uses per-scene folders and creates subfolders like:

- `rgb/`
- `vicon_data/`
- `event_cam_left_npy/` (if enabled)
- `event_cam_right_npy/` (if enabled)

Use consistent data roots so downstream scripts can find files.
