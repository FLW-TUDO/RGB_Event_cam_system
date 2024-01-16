# Transform the coordinates in RGB camera frame to event camera frame
# RGB camera is cam0, Event camera 1 is cam1 and event camera 2 is cam2
import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R

# base to camera_vicon transformation. The below values are taken from recorded json file
translation = [-1.394129520212119, -1.0855489055738885, 1.3968309243307508]
rotation_quat = [-0.11836236214272157, 0.27876023425709784, 0.28786393737803234, 0.9085248684445238]

# get rotation matrix from quaternion
rotation = R.from_quat(rotation_quat).as_matrix()
# make homogeneous transformation matrix
H_base_2_cam_vicon = np.eye(4)
H_base_2_cam_vicon[:3, :3] = rotation
H_base_2_cam_vicon[:3, 3] = translation

# make homogeneous transformation matrix from base to camera optical frame
H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)

# invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
H_cam_optical_2_vicon = np.eye(4)
H_cam_optical_2_vicon[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
H_cam_optical_2_vicon[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])


rgb_img_path = "/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam0/1704386799592976847.png"
event_cam_img = "/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam1/1704386800701017000.png"
quat_cam2_cam1 = [0.00026235, -0.00376717, -0.00030698, 0.99999282]
quat_cam1_cam0 = [-0.00916228, -0.04119687, 0.0005243, 0.9991089]
R_cam1_cam0 = R.from_quat(quat_cam1_cam0).as_matrix()

#Transformation matrix from camera 1 to camera 0
H_cam1_cam0 = np.eye(4)
H_cam1_cam0[:3, :3] = R_cam1_cam0
H_cam1_cam0[:3, 3] = [0.05834856, 0.00157235, 0.02935724]

# Transformation matrix from camera 2 to camera 1
H_cam2_cam1 = np.eye(4)
H_cam2_cam1[:3, :3] = R.from_quat(quat_cam2_cam1).as_matrix()
H_cam2_cam1[:3, 3] = [-0.10245441, 0.00011387, 0.00283878]

# project point (x,y,z) in cam0 coordinate to cam1 coordinate
point_cam0 = np.array([
    [ 1,  0,  0,  863],
    [0,  1,  0,   819],
    [ 0, 0,  1,  0],
    [ 0,  0,  0,  1]])
point_cam1 = np.matmul(H_cam1_cam0, point_cam0)
print(point_cam1)
# project point (863,819) in cam1 coordinate to cam2 coordinate
point_cam2 = np.matmul(H_cam2_cam1, point_cam1)
print(point_cam2)

# use open cv to show the rgb and event camera images
rgb_img = cv2.imread(rgb_img_path)
cv2.circle(rgb_img, (863, 819), 10, (0, 0, 255), -1)
cv2.imshow("img", rgb_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

event_img = cv2.imread(event_cam_img)
cv2.circle(event_img, (point_cam1[0,3], 819), 10, (0, 0, 255), -1)
cv2.imshow("img", event_img)
cv2.waitKey(0)
cv2.destroyAllWindows()