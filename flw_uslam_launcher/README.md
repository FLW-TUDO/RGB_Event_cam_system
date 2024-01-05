This is a ROS package containing launch files, calibration and documentation
to run ultimate SLAM using our DVXplorer camera.
The ultimate SLAM will run only using event camera and it is internal IMU

## Requirments
[e2calib package](https://github.com/uzh-rpg/e2calib) needs to be installed first to be able to
reconstruct the events to normal frame images. The scripts "convert.py" and "offline_reconstruction.py" will be used.
Install the package in directory:

     cd ~/event_camera/

Also [Kalibr toolbox](https://github.com/ethz-asl/kalibr) needs be installed. install kalibr in workspace directory "~/kalibr_workspace"
and don't the sourcing to the .bashrc. Just source its workspace before using it using this command:

     source ~/kalibr_workspace/devel/setup.bash

also install uslam in this workspace directory "~/uslam_ws". Add the workspace sourcing in the .bashrc as usual for ROS.

All calibration data will be saved in 

     cd ~/event_camera/calibration_data

Install this in a workspace and make sure the workspace is sources

## event camera calibration

Launch event camera with ROS from a launch file in this package:

     roslaunch flw_uslam_launcher event_cam_only.launch

first the event camera itself needs to be calibrated.
To do this first collect a ROS bag with event only and move the checkerboard slowly
in front of the camera covering all the image sides.

     mkdir ~/event_camera/calibration_data/events_only_calibration
     cd ~/event_camera/calibration_data/events_only_calibration
     rosbag record /dvs/events --output-name events_only.bag

Then run the "convert.py" then the "offline_reconstruction.py" scripts from the e2calib package.

     cd ~/event_camera/e2calib
     python python/convert.py --output_file ~/event_camera/calibration_data/events_only_calibration/events.h5 --topic /dvs/events ~/event_camera/calibration_data/events_only_calibration/events_only.bag
     python python/offline_reconstruction.py --h5file ~/event_camera/calibration_data/events_only_calibration/events.h5 --freq_hz 20 --output_folder ~/event_camera/calibration_data/events_only_calibration/reconstructed_event_images --use_gpu

The calibration can be performed using kalibr. Kalibr reads the images from a ROS bag.
So the reconstructed event images have to converted to a ROS bag using this command:

Note: remove sourcing of uslam workspace from the "~/.bashrc" as you won't find Kalibr if uslam is sourced. Why? IDK

     cd ~/event_camera/calibration_data/events_only_calibration/
     mv reconstructed_event_images/e2calib/ reconstructed_event_images/cam0
     source ~/kalibr_workspace/devel/setup.bash
     rosrun kalibr kalibr_bagcreater --folder reconstructed_event_images/ --output-bag images.bag
     rosrun kalibr kalibr_calibrate_cameras \
 	      --target ../checkerboard.yaml \
 	      --models pinhole-radtan \
 	      --topics /cam0/image_raw \
 	      --bag images.bag \
 	      --bag-freq 10.0

## Event IMU calibration

Install this python package:

     pip install rosbag-merge

In this calibration, the camera moves and the checkerboard is fixed.
First collect a data of the event and the IMU:

     mkdir ~/event_camera/calibration_data/events_imu_calibration
     cd ~/event_camera/calibration_data/events_imu_calibration
     rosbag record /dvs/events /dvs/imu --output-name events_imu.bag

same as above run e2calib:

     cd ~/event_camera/e2calib
     python python/convert.py --output_file ~/event_camera/calibration_data/events_imu_calibration/events_imu.h5 --topic /dvs/events ~/event_camera/calibration_data/events_imu_calibration/events_imu.bag
     python python/offline_reconstruction.py --h5file ~/event_camera/calibration_data/events_imu_calibration/events_imu.h5 --freq_hz 20 --output_folder ~/event_camera/calibration_data/events_imu_calibration/reconstructed_event_images --use_gpu

prepare bags for imu camera calibration:

     cd ~/event_camera/calibration_data/events_imu_calibration/
     mv reconstructed_event_images/e2calib/ reconstructed_event_images/cam0
     source ~/kalibr_workspace/devel/setup.bash
     rosrun kalibr kalibr_bagcreater --folder reconstructed_event_images/ --output-bag images.bag
     rosbag-merge --topics /dvs/imu /cam0/image_raw --input_paths . --output_path . --outbag_name images_imu

running Kalibr to :

     rosrun kalibr kalibr_calibrate_imu_camera \
          --target ../checkerboard.yaml \
          --bag images_imu.bag \
          --cam ../events_only_calibration/images-camchain.yaml \
          --imu /home/gouda/uslam_ws/src/rpg_ultimate_slam_open/calibration/imu/dvxplorer_BMI160.yaml \
 	      --bag-freq 1.0 \
          --verbose

## running USLAM

roslaunch ze_vio_ceres live_DAVIS240C_events_only.launch camera_name:=DVXplorer timeshift_cam_imu:=-0.00015514753826769825 #0.010361079926317417 #-0.001984034119291302

catkin config \
     --init --mkdirs --extend /opt/ros/noetic \
     --merge-devel --cmake-args \
     -DCMAKE_BUILD_TYPE=Release

     msg.linear_acceleration.x = acc_ms[0][0]*Bias_correction[0][0] + acc_ms[0][1]*Bias_correction[1][0] + acc_ms[0][2]*Bias_correction[2][0];
						msg.linear_acceleration.y = acc_ms[1][0]*Bias_correction[0][0] + acc_ms[1][1]*Bias_correction[1][0] + acc_ms[1][2]*Bias_correction[2][0];
						msg.linear_acceleration.z = acc_ms[2][0]*Bias_correction[0][0] + acc_ms[2][1]*Bias_correction[1][0] + acc_ms[2][2]*Bias_correction[2][0];


Bias of gyroscope are changedd in frontend_base.cpp.
Acceleremeter values are corrected in driver.cpp

Execute C++ scipts
g++ test_camera.cpp -o test_camera $(pkg-config --cflags --libs /usr/lib/x86_64-linux-gnu/pkgconfig/opencv4.pc)
./test_camera
