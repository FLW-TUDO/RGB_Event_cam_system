⬅️ Back to [Home](index.md)
# Troubleshooting

## Calibration Issues

- Kalibr optimization failure: increase `timeOffsetPadding` in `kalibr_calibrate_imu_camera.py` (see `README.md`).

## Camera Not Detected

- Verify drivers are installed for your camera hardware.
- For IDS cameras, follow `ids_camera_driver/README.md` and reload udev rules.

## Missing ROS Topics

- Ensure the correct workspace is sourced.
- Verify camera nodes are running before recording bags.
