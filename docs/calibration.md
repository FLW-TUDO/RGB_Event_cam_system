⬅️ Back to [Home](index.md)

# Calibration Workflow (RGB + Stereo Event Cameras)

This page describes the recommended calibration workflow for a synchronized setup:
- **cam0**: RGB camera
- **cam1**: left event camera (reconstructed frames)
- **cam2**: right event camera (reconstructed frames)

The workflow uses:
- **e2calib** for converting event streams and reconstructing intensity-like frames
- **Kalibr** for multi-camera calibration

---

## Prerequisites

- Ubuntu 20.04 + ROS Noetic installed and sourced
- `e2calib` installed and accessible
- `kalibr` workspace built (see Installation page)
- Checkerboard / calibration target YAML available (e.g., `checkerboard.yaml`)

---

## Recommended Directory Layout

This guide assumes the following base directory:

- `~/event_camera/calibration_data/RGB_stereo_event/`

If you use a different directory, update the paths consistently.

---

## Step 1 — Record a Calibration ROS Bag

**Goal:** capture synchronized RGB + stereo events.

Keep the cameras stationary and move the checkerboard so it covers the full field of view
(near/far, corners, different orientations).

```bash
mkdir -p ~/event_camera/calibration_data/events_only_calibration
cd ~/event_camera/calibration_data/events_only_calibration

rosbag record \
  /dvxplorer_left/events \
  /dvxplorer_right/events \
  /rgb/image_raw \
  --output-name events_only.bag
````

**Output:** `events_only.bag`

---

## Step 2 — Convert Event Topics to H5

Convert the recorded event topics into `.h5` files using `e2calib`.

```bash
cd ~/event_camera/e2calib

python /home/eventcamera/e2calib/python/convert.py \
  --input_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_only.bag \
  --topic '/dvxplorer_left/events' \
  --output_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_left.h5

python /home/eventcamera/e2calib/python/convert.py \
  --input_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_only.bag \
  --topic '/dvxplorer_right/events' \
  --output_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_right.h5
```

**Outputs:**

* `events_left.h5`
* `events_right.h5`

---

## Step 3 — Reconstruct Event Frames

Reconstruct intensity-like frames from the event streams.

```bash
python /home/eventcamera/e2calib/python/offline_reconstruction.py \
  --h5file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_right.h5 \
  --freq_hz 90 \
  --output_folder /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images \
  --use_gpu

mv /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/e2calib/ \
   /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam2


python /home/eventcamera/e2calib/python/offline_reconstruction.py \
  --h5file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_left.h5 \
  --freq_hz 90 \
  --output_folder /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images \
  --use_gpu

mv /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/e2calib/ \
   /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam1
```

**Outputs:**

* `reconstructed_event_images/cam1/` (left event camera frames)
* `reconstructed_event_images/cam2/` (right event camera frames)

**Notes:**

* `--freq_hz 90` should match your reconstruction needs (higher frequency = more frames, more computation).
* If you do not have a GPU, remove `--use_gpu` (slower).

---

## Step 4 — Extract RGB Frames

Extract RGB frames from the bag into a `cam0` folder.

```bash
python RGB_Event_cam_system/calibration_rgb_event_cameras_extrinsics/extract_rgb_img_from_bag.py
```

**Output:** `cam0/` (RGB frames)

> Ensure the script writes into the same calibration folder hierarchy used above.

---

## Step 5 — Create an Image Bag for Kalibr

Kalibr expects camera images in a ROS bag. Convert the reconstructed image folders into an image bag:

```bash
source ~/kalibr_workspace/devel/setup.bash
cd /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event

rosrun kalibr kalibr_bagcreater \
  --folder reconstructed_event_images/ \
  --output-bag images.bag
```

**Output:** `images.bag`

---

## Step 6 — Run Kalibr Multi-Camera Calibration

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --target ../checkerboard.yaml \
  --models pinhole-radtan pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw /cam2/image_raw \
  --bag images.bag \
  --bag-freq 100.0 \
  --verbose
```

### Lens / Distortion Model Note (DVXplorer)

Depending on the DVXplorer lens, an omnidirectional model may be required (e.g., `omni-radtan`).
If your residuals are high or calibration fails consistently, try switching the event camera model.

---

## Troubleshooting

### Kalibr “Optimization failed”

If Kalibr reports optimization failures, increase `timeOffsetPadding` in
`kalibr_calibrate_imu_camera.py`.

Trade-off:

* Higher value → more robust, but slower
* Too low → optimizer can cross knot boundaries and fail

(After changing the script, rerun the calibration.)
