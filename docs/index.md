# RGB + Stereo Event Camera System Documentation

This documentation describes installation, configuration, calibration and
data processing workflows for the RGB + stereo event camera system.

The guide is intended for engineers, researchers and industry partners
requiring a reliable and reproducible setup.

---

## System Architecture

The system integrates:

- RGB camera for intensity imaging
- Stereo event cameras for high-temporal-resolution sensing
- ROS-based synchronization
- Event reconstruction pipeline
- Multi-camera spatial calibration

---

## Documentation Structure

1. [Installation](installation.md)
2. [ROS Workspace Setup](ros-workspace.md)
3. [Calibration Workflow](calibration.md)
4. [Data Extraction](data-extraction.md)
5. [Dataset Format](dataset-format.md)
6. [Troubleshooting](troubleshooting.md)

---

## Recommended Workflow

1. Install dependencies and toolchains
2. Configure ROS workspaces
3. Perform event reconstruction
4. Execute multi-camera calibration
5. Validate extrinsics
6. Record and export datasets

---

## Tested Hardware Configuration

- 2 × DVXplorer event cameras
- 1 × IDS RGB camera
- Intel-based workstation
- NVIDIA GPU (optional, for accelerated reconstruction)

---