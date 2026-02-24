⬅️ Back to [Home](index.md)
# ROS Workspace Conventions

This project assumes a standard ROS Noetic catkin workspace.

## Recommended Layout

```
~/event_camera_ws
├─ src/
│  ├─ RGB_Event_cam_system
│  └─ other packages...
└─ devel/
```

## Sourcing

Before running any scripts or launch files, source your workspace:

```bash
source ~/event_camera_ws/devel/setup.bash
```

If you use Kalibr or other toolboxes, source them only when needed to avoid environment conflicts.

## Launch Files

The main ROS launch file is:

- `RGB_event_cam_stereo.launch`

```bash
roslaunch RGB_event_cam_stereo.launch.launch
```

This should bring up the RGB + two event cameras and publish image/event topics.
