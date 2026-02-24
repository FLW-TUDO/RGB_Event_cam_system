# RGB + Stereo Event Camera System (ROS Noetic)

This repository provides a complete workflow for operating, calibrating and
using a synchronized RGB + stereo event camera system under ROS Noetic.

---

## System Overview

The setup consists of:

- 1 × RGB camera (IDS)
- 2 × Stereo event cameras (DVXplorer)
- Ubuntu 20.04
- ROS Noetic

Supported functionality:

- Event stream reconstruction using e2calib
- Multi-camera spatial calibration using Kalibr

---

## Supported Environment

| Component | Version |
|------------|----------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| Event Camera | DVXplorer |
| RGB Camera | IDS (uEye) |
| Calibration Toolbox | Kalibr |
| Event Reconstruction | e2calib |


## Documentation

Complete setup and calibration instructions are available here:

➡️ **[Full Documentation](docs/index.md)**

Documentation includes:

- Installation and dependencies
- ROS workspace setup
- Camera calibration workflow
- Troubleshooting guide

---