# Calibration Workflow

This workflow is based on the steps in `README.md`.

## 1) Record a Calibration Bag

Keep cameras stationary and move a checkerboard across the full field of view.

```bash
mkdir -p ~/event_camera/calibration_data/events_only_calibration
cd ~/event_camera/calibration_data/events_only_calibration
rosbag record /dvxplorer_left/events /dvxplorer_right/events /rgb/image_raw --output-name events_only.bag
```

## 2) Convert Events to H5

```bash
cd ~/event_camera/e2calib
python /home/eventcamera/e2calib/python/convert.py --output_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_left.h5 --topic '/dvxplorer_left/events' --input_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_only.bag

python /home/eventcamera/e2calib/python/convert.py --output_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_right.h5 --topic '/dvxplorer_right/events' --input_file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_only.bag
```

## 3) Reconstruct Event Images

```bash
python /home/eventcamera/e2calib/python/offline_reconstruction.py --h5file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_right.h5 --freq_hz 90 --output_folder /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images --use_gpu
mv /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/e2calib/ /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam2

python /home/eventcamera/e2calib/python/offline_reconstruction.py --h5file /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/events_left.h5 --freq_hz 90 --output_folder /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images --use_gpu
mv /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/e2calib/ /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam1
```

## 4) Extract RGB Images

```bash
python RGB_Event_cam_system/calibration_rgb_event_cameras_extrinsics/extract_rgb_img_from_bag.py
```

RGB frames are stored in `cam0`.

## 5) Run Kalibr

```bash
source ~/kalibr_workspace/devel/setup.bash
cd /home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event

rosrun kalibr kalibr_bagcreater --folder reconstructed_event_images/ --output-bag images.bag

rosrun kalibr kalibr_calibrate_cameras \
  --target ../checkerboard.yaml \
  --models pinhole-radtan pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw /cam2/image_raw \
  --bag images.bag \
  --bag-freq 100.0 \
  --verbose
```

Note: The DVXplorer lens model may be `omni-radtan`, depending on the lens.

## Common Calibration Error

If Kalibr reports optimization failures, increase `timeOffsetPadding` in `kalibr_calibrate_imu_camera.py` as noted in `README.md`.
