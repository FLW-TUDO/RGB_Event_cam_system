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

H_cam_vicon_2_cam_optical = np.array([
    [0.1844511, 0.10184152, 0.97755107, 0.13975843],
    [-0.98279335, 0.02897629, 0.18242149, -0.54963646],
    [-0.00974772, -0.99437854, 0.10543387, -0.50285077],
    [0., 0., 0., 1.]])

params = [1.93686226e+03, 1.93330361e+03, 9.85688080e+02, 7.71096964e+02]
# params = [2592.7798180209766, 2597.1074116646814, 1121.2441077660412, 690.1066893999352]
camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
# distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])
# camera_matrix = np.array([[1.93686226e+03, 0.00000000e+00, 9.85688080e+02],
#                          [0.00000000e+00, 1.93330361e+03, 7.71096964e+02],
#                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion_coefficients = np.array([-0.12905658, -0.01019267, 0.00562304, -0.00015354, 0.13542021])

img_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/third_calib/test/images/0.png'

img_test = cv2.imread(img_path)
img_test_cam1 = cv2.imread(img_path)
img_test_cam2 = cv2.imread(img_path)

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

# make point of verification
H_v_2_point = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]])
# transform the point to the camera optical frame
H_cam_optical_2_point = np.matmul(H_cam_optical_2_vicon, H_v_2_point)

# get the 3d point
t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
t_rgb_2_point = t_cam_optical_2_point
H_rgb_2_point = H_cam_optical_2_point

H_cam1_rgb = np.array([[0.9966050852350032, 0.0018025810956462127, 0.08231072096103041, 0.058348558249773974],
                        [-0.00029275262105358537, 0.9998315556310521, -0.018351421206337717, 0.0015723475769044038],
                        [-0.0823299361085253, 0.018265023016223507, 0.9964377404306713, 0.02935723981670002],
                        [0.0, 0.0, 0.0, 1.0]])
H_cam2_cam1 = np.array([[0.9999714283197217, -0.0006159292736051988, 0.007534134014844326, -0.10245440610370138],
                        [0.0006119760079932377, 0.9999996738738459, 0.0005270081360818972, 0.0001138693144027686],
                        [-0.007534456157504587, -0.0005223823693158363, 0.9999714791368196, 0.0028387840213753655],
                        [0.0, 0.0, 0.0, 1.0]])
#======================================================================================
# Transform the point from rgb to cam 1 frame
#======================================================================================
H_cam_1_point = np.matmul(H_cam1_rgb, H_rgb_2_point)
t_cam1_2_point = H_cam_1_point[:3, 3]
# cam1 parameters
camera_mtx_cam1 = np.array([[704.7619734668378, 0, 300.78994093351923], [0, 705.9081489448346, 229.8585626133938], [0, 0, 1]])
distortion_coeffs_cam1 = np.array([-0.364714404546865, 0.07816135096611694, 0.005085888890347796, -0.0036418328217707836])

# Project 3D points to image plane
points_2d_cam1, _ = cv2.projectPoints(np.array([t_cam1_2_point]), np.array([[0, 0, 0]]), np.array([[0, 0, 0]]), camera_mtx_cam1, distortion_coeffs_cam1)
points_2d_cam1 = np.round(points_2d_cam1[0]).astype(int)
print(points_2d_cam1)
# Display the 2d points on the image
img_test = cv2.circle(img_test_cam1, tuple(points_2d_cam1[0][0]), 10, (255, 0, 0), -1)
cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5)) # resize image to 0.5 for display
cv2.waitKey(0)

#======================================================================================
# Transform the point from cam 1 to cam 2 frame
#======================================================================================
H_cam_2_point = np.matmul(H_cam2_cam1, H_cam_1_point)
t_cam2_2_point = H_cam_2_point[:3, 3]
# cam2 Parameters
camera_mtx_cam2 = np.array([[2.6174837933891757, 0, 2644.449052050593], [0, 2655.684777520004, 308.2314261541802], [0, 0, 1]])
distortion_coeffs_cam2 = np.array([0.4008405051407828, -35.782378911149834, 0.018867056305151703, -0.0068405689007467185])

# Project 3D points to image plane
points_2d_cam2, _ = cv2.projectPoints(np.array([t_cam2_2_point]), np.array([[0, 0, 0]]), np.array([[0, 0, 0]]), camera_mtx_cam2, distortion_coeffs_cam2)
points_2d_cam2 = np.round(points_2d_cam2[0]).astype(int)
print(points_2d_cam2)
# Display the 2d points on the image
img_test = cv2.circle(img_test_cam2, tuple(points_2d_cam2[0][0]), 10, (255, 0, 0), -1)
cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5)) # resize image to 0.5 for display
cv2.waitKey(0)

cv2.destroyAllWindows()



