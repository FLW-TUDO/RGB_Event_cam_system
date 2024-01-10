import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R


# make homogeneous transformation matrix
#H_cam_vicon_2_cam_optical = np.eye(4)
#H_cam_vicon_2_cam_optical[:3, :3] = np.array([[]])
#H_cam_vicon_2_cam_optical[:3, 3] = np.array([[]])
H_cam_vicon_2_cam_optical = np.array([[0.45268017,  0.71338164, -0.53494607,  1.81758212],
                             [ 0.43286333,  0.34868801,  0.83129177, -1.42294582],
                             [ 0.77955757, -0.60786784, -0.1509526,   1.84091085],
                             [ 0,          0,          0,          1        ]])
# make sample expected values for the translation matrix from vicon camera to vicon optical frame
#H_cam_vicon_2_cam_optical = np.array([
#    [ 0,   0,  1,  0.03],
#    [-1,   0,  0,  0.02],
#    [ 0,  -1,  0,  -0.02],
#    [ 0,   0,  0,  1]])

checker_board_size = (10, 7)
square_size_cm = 5
#params = [2592.7798180209766, 2597.1074116646814, 1121.2441077660412, 690.1066893999352]
#distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])
camera_matrix = np.array([[1.93686226e+03, 0.00000000e+00, 9.85688080e+02],
                [0.00000000e+00, 1.93330361e+03, 7.71096964e+02],
                 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion_coefficients = np.array([-0.12905658, -0.01019267, 0.00562304, -0.00015354, 0.13542021])
#camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])

img_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/first_calib/test/images/0.png'

img_test = cv2.imread(img_path)

# vicon to camera vicon transformation
translation = [-0.3071228315799955, -2.144467683991964, 1.1205968606727794]
rotation_quat = [0.720806440523476, -0.15531550691984375, 0.17980399921105955, 0.6511418359142791]

# get rotation matrix from quaternion
rotation = R.from_quat(rotation_quat).as_matrix()
# make homogeneous transformation matrix
H_vicon_2_cam_vicon = np.eye(4)
H_vicon_2_cam_vicon[:3, :3] = rotation
H_vicon_2_cam_vicon[:3, 3] = translation

# make homogeneous transformation matrix from vicon to camera optical frame
H_vicon_2_cam_optical = np.matmul(H_vicon_2_cam_vicon, H_cam_vicon_2_cam_optical)

# invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
H_cam_optical_2_vicon = np.eye(4)
H_cam_optical_2_vicon[:3, :3] = np.transpose(H_vicon_2_cam_optical[:3, :3])
H_cam_optical_2_vicon[:3, 3] = -np.matmul(np.transpose(H_vicon_2_cam_optical[:3, :3]), H_vicon_2_cam_optical[:3, 3])

# make point of verification
H_v_2_point = np.array([
    [ 1,  0,  0,  1],
    [0,  1,  0,  -0.3],
    [ 0, 0,  1,  0],
    [ 0,  0,  0,  1]])
# transform the point to the camera optical frame
H_cam_o_2_point = np.matmul(H_v_2_point, H_cam_optical_2_vicon)
# get the 3d point
t_cam_o_2_point = H_cam_o_2_point[:3, 3]

# Project 3D points to image plane
points_2d = cv2.projectPoints(t_cam_o_2_point, np.eye(3), np.zeros(3), camera_matrix, distortion_coefficients)
# get pixel values and round them
points_2d = np.round(points_2d[0]).astype(int)
print(points_2d)
# Display the 2d points on the image
img_test = cv2.circle(img_test, tuple(points_2d[0][0]), 10, (255, 0, 0), -1)
# resize image to 0.5 for display
img_test = cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5)
cv2.imshow('img', img_test)
cv2.waitKey(0)
cv2.destroyAllWindows()

