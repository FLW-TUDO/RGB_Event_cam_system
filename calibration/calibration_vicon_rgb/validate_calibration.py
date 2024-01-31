import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R

# make sample expected values for the translation matrix from vicon camera to vicon optical frame
# H_cam_vicon_2_cam_optical = np.array([
#    [ 0,   0,  1,  0.03],
#    [-1,   0,  0,  0.04],
#    [ 0,  -1,  0,  -0.045],
#    [ 0,   0,  0,  1]])

H_cam_vicon_2_cam_optical = np.array([[-0.00653897, 0.01742515, 0.99982679, 0.06225329],
                                      [-0.99986245, -0.01535384, -0.00627162, 0.00494558],
                                      [0.01524189, -0.99973028, 0.01752315, -0.03356527],
                                      [0., 0., 0., 1.]])
H_cam_optical_2_cam_vicon = np.linalg.inv(H_cam_vicon_2_cam_optical)

params = [1.81601107e+03, 1.81264445e+03, 1.00383169e+03, 7.16010695e+02]
# params = [1.93686226e+03, 1.93330361e+03, 9.85688080e+02, 7.71096964e+02]
# params = [2592.7798180209766, 2597.1074116646814, 1121.2441077660412, 690.1066893999352]
camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
# distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])
# camera_matrix = np.array([[1.93686226e+03, 0.00000000e+00, 9.85688080e+02],
#                          [0.00000000e+00, 1.93330361e+03, 7.71096964e+02],
#                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion_coefficients = np.array([-1.76581808e-01, 1.06210912e-01, -1.55074994e-04,
                                    5.03366350e-04, -4.07696624e-02])

img_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/third_calib/test/images/0.png'

img_test = cv2.imread(img_path)

# base to camera_vicon transformation. The below values are taken from recorded json file
translation = [1.7316599964996864, -0.5580612164401452, 0.8022367272635139]
rotation_quat = [-0.18889019791401296, 0.04653582971135208, 0.9491809235562345, 0.24740752624367504]

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

# make point of verification
H_v_2_point = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 2],
    [0, 0, 1, 0],
    [0, 0, 0, 1]])
# transform the point to the camera optical frame
H_cam_optical_2_point = np.matmul(H_cam_optical_2_vicon, H_v_2_point)

# H_point_2_cam_o = np.eye(4)
# H_point_2_cam_o[:3, :3] = np.transpose(H_cam_o_2_point[:3, :3])
# H_point_2_cam_o[:3, 3] = -np.matmul(np.transpose(H_cam_o_2_point[:3, :3]), H_cam_o_2_point[:3, 3])
# get the 3d point
t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
# t_point_2_cam_o = H_point_2_cam_o[:3, 3]

# Project 3D points to image plane
points_2d = cv2.projectPoints(t_cam_optical_2_point, np.eye(3), np.zeros(3), camera_matrix, distortion_coefficients)
# get pixel values and round them
points_2d = np.round(points_2d[0]).astype(int)
print(points_2d)
# Display the 2d points on the image
img_test = cv2.circle(img_test, tuple(points_2d[0][0]), 10, (255, 0, 0), -1)

cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))  # resize image to 0.5 for display
cv2.waitKey(0)
cv2.destroyAllWindows()
