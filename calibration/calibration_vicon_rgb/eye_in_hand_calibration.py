"""
Eye in hand calibration
this script that takes images of checkerboard and a robot pose and output camera calibration
"""

import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R


data_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration'
checker_board_size = (10, 7)
params = [2592.7798180209766, 2597.1074116646814, 1121.2441077660412, 690.1066893999352]
distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])

json_path = os.path.join(data_path, 'vicon_coordinates.json')
camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])

images_path = os.path.join(data_path, 'images')
num_of_images = len(os.listdir(images_path))

objp = np.zeros((checker_board_size[0]*checker_board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checker_board_size[0], 0:checker_board_size[1]].T.reshape(-1, 2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# iterate over images to get checkboard transformation
for img_id in range(num_of_images):
    img_path = os.path.join(images_path, str(img_id)+'.png')
    img = cv2.imread(img_path)

    # undistort image using camera matrix and distortion coefficients

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, checker_board_size, None)
    # if found, add object points, image points (after refining them)
    if ret == True:
        # refine corners
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        # draw and display the corners
        img = cv2.drawChessboardCorners(img, checker_board_size, corners2, ret)
        #cv2.imshow('img', img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        objpoints.append(objp)
        imgpoints.append(corners2)

# calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], camera_matrix, distortion_coefficients)

# TODO we need to have tvects in meters

# TODO check if we need the inverse of the transformation

# convert rvecs from Rodrigues to rotation matrix
rvecs = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]

# rename rvecs and tvecs to be consistent with the paper (from tuple to np.array)
R_c_t_vecs = np.array(list(rvecs))
t_c_t_vecs = np.array(list(tvecs))

# get transformation from base to gripper
# read the json file
with open(json_path, 'r') as f:
    data = json.load(f)
R_b_g_vecs = []
t_b_g_vecs = []
for i in range(len(data)):
    translation = data[str(i)]['translation']
    rotation = data[str(i)]['rotation']
    R_g_b_quaternion = np.array(rotation)
    # convert quaternion to rotation matrix using Scipy
    R_g_b = R.from_quat(R_g_b_quaternion).as_matrix()

    t_g_b = np.array(translation)

    R_b_g = np.transpose(R_g_b)
    # convert rotation matrix to Rodrigues
    #R_b_g, _ = cv2.Rodrigues(R_b_g)
    t_b_g = -np.matmul(np.transpose(R_g_b), t_g_b)
    R_b_g_vecs.append(R_b_g)
    t_b_g_vecs.append(t_b_g)
R_b_g_vecs = np.array(R_b_g_vecs)
t_b_g_vecs = np.array(t_b_g_vecs)

# calculate the eye in hand calibration
R_g_c, t_g_c = cv2.calibrateHandEye(R_b_g_vecs, t_b_g_vecs, R_c_t_vecs, t_c_t_vecs, cv2.CALIB_HAND_EYE_TSAI)

print('R_g_c: ', R_g_c)
print('t_g_c: ', t_g_c)


