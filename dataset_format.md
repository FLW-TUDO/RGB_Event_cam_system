# Dataset Format

Directory Structure:
```
DATASET_NAME
├─ cameras_params.json
├─ static_transformations.json
├─ transformations.json
├─ rgb
├─ event_camera_left
├─ event_camera_right
```

cameras_params.json
```
{
  "RGB":                { "intrinsic": [fx fy cx cy] "distortion": [k1 k2 k3 k4]},
  "event_camera_left":  { "intrinsic": [fx fy cx cy] "distortion": [k1 k2 k3 k4]},
  "event_camera_right":  { "intrinsic": [fx fy cx cy] "distortion": [k1 k2 k3 k4]},
}
```

static_transformations.json
```
{
        "rgb_cam_optical_to_event_1": {"translation": [], "rotation": []},
        "rgb_cam_optical_to_event_2": {"translation": [], "rotation": []}
}
```

transformations.json
```
{
        "0":  {
                "vicon_to_cam_optical": {"translation": [], "rotation": []},
                "vicon_to_object": {"translation": [], "rotation": [], "obj_id": 0}
                "time_stamp": 1706115138000000000
        }
        ...
}
```

RGB folder
```
rgb
├─ 0.png
├─ 1.png
├─ .....
```

Event camera folders
```
event_camera_left
├─ Pose.json
├─ BBox.json
├─ 1706115139000000000.npy
├─ 1706115143000000000.npy
├─ .....

event_camera_right
├─ Pose.json
├─ BBox.json
├─ 1706115140000000000.npy
├─ 1706115144000000000.npy
├─ .....
```
```
Pose.json
[timestamp,[tx, ty, tz, rx, ry, rz]]
.....

BBox.json
{"timestamp": timestamp, "xmin": , "xmax": , "ymin": , "ymax":}
...
```

