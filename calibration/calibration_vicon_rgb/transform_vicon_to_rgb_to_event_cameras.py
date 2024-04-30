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
params = [1.81601107e+03, 1.81264445e+03, 1.00383169e+03, 7.16010695e+02]
# params = [2592.7798180209766, 2597.1074116646814, 1121.2441077660412, 690.1066893999352]
camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
# distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])
# camera_matrix = np.array([[1.93686226e+03, 0.00000000e+00, 9.85688080e+02],
#                          [0.00000000e+00, 1.93330361e+03, 7.71096964e+02],
#                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion_coefficients = np.array([-1.76581808e-01, 1.06210912e-01, -1.55074994e-04,
                                    5.03366350e-04, -4.07696624e-02])

#img_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/third_calib/test2/images/0.png'
img_path = "/home/eventcamera/data/rgb/1712920771350731701.png"
img_path_left = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/third_calib/test2/reconstructed_images/hall2/left/1706718612728642000.png'
img_path_right = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/third_calib/test2/reconstructed_images/hall1/right/1706718567179738000.png'
img_path_left = '/home/eventcamera/data/reconstructed_images/event_cam_left/1712920771359574000.png'
img_path_right = '/home/eventcamera/data/reconstructed_images/event_cam_right/1712920771358463000.png'

img_test = cv2.imread(img_path)
img_test_cam1 = cv2.imread(img_path_left)
img_test_cam2 = cv2.imread(img_path_right)

translation = []
rotation_quat = []
# base to camera_vicon transformation. The below values are taken from recorded json file
json_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/third_calib/test2/vicon_coordinates.json'
json_path = '/home/eventcamera/data/vicon_data/test.json'
with open(json_path, 'r') as f:
    data = json.load(f)
for i in range(len(data)):
    translation = data[str(i)]['translation']
    rotation_quat = data[str(i)]['rotation']

# get rotation matrix from quaternion
rotation = R.from_quat(rotation_quat).as_matrix()
# make homogeneous transformation matrix
H_base_2_cam_vicon = np.eye(4)
H_base_2_cam_vicon[:3, :3] = rotation
H_base_2_cam_vicon[:3, 3] = translation

# make homogeneous transformation matrix from base to camera optical frame
H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)

# invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
H_cam_optical_2_base = np.eye(4)
H_cam_optical_2_base[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])

# make point of verification -0.09324468209902101,
#       0.09552323989025352,
#       0.21876072081161987
H_v_2_point = np.array([
    [1, 0, 0, -0.09324468209902101],
    [0, 1, 0, 0.09552323989025352],
    [0, 0, 1, 0.21876072081161987],
    [0, 0, 0, 1]])
# transform the point to the camera optical frame
H_cam_optical_2_point = np.matmul(H_cam_optical_2_base, H_v_2_point)

# get the 3d point
t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
print(t_cam_optical_2_point)
points_2d = cv2.projectPoints(t_cam_optical_2_point, np.eye(3), np.zeros(3), camera_matrix, distortion_coefficients)
# get pixel values and round them
points_2d = np.round(points_2d[0]).astype(int)
print(points_2d)
# Display the 2d points on the image
img_test = cv2.circle(img_test, tuple(points_2d[0][0]), 10, (255, 0, 0), -1)

cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))  # resize image to 0.5 for display
cv2.waitKey(0)

# t_rgb_2_point = t_cam_optical_2_point
H_rgb_2_point = H_cam_optical_2_point

H_cam1_2_rgb = np.array([[0.9971993087878418, 0.0005258497031806043, 0.07478811426377688, 0.053269788085482585],
                       [0.0006902602101232752, 0.9998679823207766, -0.016233960410603578, 0.009952490862473908],
                       [-0.07478677753376195, 0.016240117359809583, 0.99706729787625, -0.04082399413941046],
                       [0.0, 0.0, 0.0, 1.0]])

H_cam2_cam1 = np.array([[0.9999312261471128, -0.00039260427945142073, 0.011721298468554263, -0.10223038360927061],
                        [0.00039304051036216355, 0.9999999221500911, -3.491341559211893e-05, 0.00027606541625718685],
                        [-0.011721283848896005, 3.9517959594199465e-05, 0.9999313026119555, 0.005272335654555953],
                        [0.0, 0.0, 0.0, 1.0]])
# ======================================================================================
# Transform the point from rgb to cam 1 frame
# ======================================================================================
H_cam1_2_point = np.matmul(H_cam1_2_rgb, H_rgb_2_point)
t_cam1_2_point = H_cam1_2_point[:3, 3]
print(t_cam1_2_point)
# cam1 parameters
camera_mtx_cam1 = np.array(
    [[718.9289498879248, 0, 287.4206641081329], [0, 718.8476596505732, 232.6402787336837], [0, 0, 1]])
distortion_coeffs_cam1 = np.array(
    [-0.3094967913882128, -0.10722657430965295, 0.008512403913427787, 0.000592616793055609])

# Project 3D points to image plane
points_2d_cam1 = cv2.projectPoints(np.array([t_cam1_2_point]), np.eye(3), np.zeros(3), camera_mtx_cam1,
                                   distortion_coeffs_cam1)
points_2d_cam1 = np.round(points_2d_cam1[0]).astype(int)
print(points_2d_cam1)
# Display the 2d points on the image
img_test = cv2.circle(img_test_cam1, tuple(points_2d_cam1[0][0]), 5, (255, 0, 0), -1)
cv2.imshow('img', img_test)  # resize image to 0.5 for display
cv2.waitKey(0)

# ======================================================================================
# Transform the point from cam 1 to cam 2 frame
# ======================================================================================
H_cam2_2_point = np.matmul(H_cam2_cam1, H_cam1_2_point)
t_cam2_2_point = H_cam2_2_point[:3, 3]
print(t_cam2_2_point)
# cam2 Parameters
# camera_mtx_cam2 = np.array([[2.6174837933891757, 0, 2644.449052050593], [0, 2655.684777520004, 308.2314261541802], [0, 0, 1]])
camera_mtx_cam2 = np.array(
    [[745.1353300950308, 0,  291.1070763508334], [0, 747.3744176138202, 245.89026445203564], [0, 0, 1]])
distortion_coeffs_cam2 = np.array(
    [-0.2496983957244161, -0.2978060510925673, 0.009342174824708725, 0.00328860522240014])

# Project 3D points to image plane
points_2d_cam2 = cv2.projectPoints(np.array([t_cam2_2_point]), np.eye(3), np.zeros(3), camera_mtx_cam2,
                                   distortion_coeffs_cam2)
points_2d_cam2 = np.round(points_2d_cam2[0]).astype(int)
print(points_2d_cam2)
# Display the 2d points on the image
img_test = cv2.circle(img_test_cam2, tuple(points_2d_cam2[0][0]), 5, (255, 0, 0), -1)
cv2.imshow('img', img_test)  # resize image to 0.5 for display
cv2.waitKey(0)

cv2.destroyAllWindows()
