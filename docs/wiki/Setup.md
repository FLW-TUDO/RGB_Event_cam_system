# Setup (Ubuntu 20.04 + ROS Noetic)

This page provides a clean, end-to-end setup for the RGB + event camera pipeline. Hardware-specific steps (IDS camera) are included, because the project integrates them in the main workflow.

## 1) System Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+ (matches tooling in this repo)
- Cameras: 1 RGB + 2 event cameras

## 2) ROS Workspace

Create a ROS workspace and clone this repository into `src/`.

```bash
mkdir -p ~/event_camera_ws/src
cd ~/event_camera_ws/src
# git clone <your-repo-url>
cd ..
catkin_make
source devel/setup.bash
```

If you use multiple workspaces, ensure the correct one is sourced in your shell before running scripts.

## 3) Calibration Tool

Required for calibration and event reconstruction.

### e2calib
Used for event reconstruction to image frames.

```bash
cd ~/event_camera
# git clone https://github.com/uzh-rpg/e2calib
```

### Kalibr toolbox
Used for multi-camera calibration.

```bash
# Example workspace
mkdir -p ~/kalibr_workspace/src
cd ~/kalibr_workspace/src
# git clone https://github.com/ethz-asl/kalibr
cd ..
catkin_make
source ~/kalibr_workspace/devel/setup.bash
```

### SLAM (Future use)

## 4) IDS Camera Driver

If you use IDS cameras, install the IDS peak SDK and Python bindings using the guide in:

- `ids_camera_driver/README.md`
- https://en.ids-imaging.com/download-details/AB02491.html?os=linux_arm&version=v8&bus=64&floatcalc=hard

### 4.1) Install IDS Software Suite (uEye required)

```bash
cd ~/Downloads
# Extract the software suite and uEye driver packages
tar -xvzf ids-software-suite-linux-<arch>-4.96.1-debian.tgz
tar -xvf ueye_4.96.1.2054_<arch>.deb.tar.gz

# Install uEye drivers
sudo dpkg -i ueye-*.deb
sudo apt -f install
```

Replace `<arch>` with `x86_64` or `arm64`.

### 4.2) Install IDS peak SDK

```bash
sudo apt update
sudo apt install \
  libqt5core5a libqt5gui5 libqt5widgets5 libqt5multimedia5 libqt5quick5 \
  qml-module-qtquick-window2 qml-module-qtquick2 \
  qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qtquick-layouts \
  qml-module-qt-labs-settings qml-module-qt-labs-folderlistmodel \
  qtbase5-dev qtdeclarative5-dev \
  libusb-1.0-0 libatomic1

# Use the exact .deb filename you downloaded
sudo apt install ./ids-peak-with-ueyetl-linux-*-<version>.deb

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4.3) Verify the SDK

```bash
ids_peak_cockpit
```

### 4.4) Install Python bindings

```bash
cd /usr/local/share/ids/bindings/python/wheel/
python3 --version
# pick the wheel that matches your Python version
python3 -m pip install ids_peak-2.4.0-cp310-cp310-linux_x86_64.whl
python3 -m pip install ids_peak_ipl-2.4.0-cp310-cp310-linux_x86_64.whl
python3 -m pip install ids_peak_afl-2.4.0-cp310-cp310-linux_x86_64.whl
```

Use the `cp38` wheels for Python 3.8.

## 5) Environment Variables and Paths

Several scripts expect data under a shared root (examples include `/mnt/smbshare/` and `/media/eventcamera/...`).

Decide a single data root (for example, `/mnt/smbshare/`) and use it consistently in:

- `extract_rgb_events_vicon_data_from_bag.py`

## 6) Verify Cameras in ROS

Once drivers are running, confirm topics exist (example):

- `/camera/image_raw`
- `/dvxplorer_left/events`
- `/dvxplorer_right/events`

Use `rostopic list` and `rostopic echo` to verify.

```bash
rostopic list
```
