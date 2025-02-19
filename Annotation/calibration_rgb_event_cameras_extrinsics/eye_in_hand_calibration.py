"""
Eye in hand calibration
this script that takes images of checkerboard and a robot pose and output camera calibration
"""

import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R

# =============================================================================
#############  Define Paths and Parameters #############
# =============================================================================
square_size_meter = 0.125
image_size = (2048, 1536)
# Third calib is the original calibration as on 20 Aug 2024
camera_param_path = '/home/eventcamera/RGB_Event_cam_system/Annotation/camera_params.json'
data_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/sixth_calib/'
checker_board_size = (8, 5)
#params = [1860.9939429743338, 1862.40872909852, 917.1768824325476, 679.5399523739977]
#distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])
# =============================================================================

json_path = os.path.join(data_path, 'vicon_coordinates.json')
images_path = os.path.join(data_path, 'images')
num_of_images = len(os.listdir(images_path))
###############

objp = np.zeros((checker_board_size[0] * checker_board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checker_board_size[0], 0:checker_board_size[1]].T.reshape(-1, 2)
objp = objp * square_size_meter

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#i = 0
# iterate over images to get checkerboard transformation
for img_id in range(num_of_images):
    img_path = os.path.join(images_path, str(img_id) + '.png')
    img = cv2.imread(img_path)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, checker_board_size, None)

    # if found, add object points, image points (after refining them)
    if ret == True:
        # refine corners
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # draw and display the corners
        img = cv2.drawChessboardCorners(img, checker_board_size, corners2, ret)
        # draw image resized to fit screen
        #cv2.imshow('img', cv2.resize(img, (0, 0), fx=0.5, fy=0.5))
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        objpoints.append(objp)
        imgpoints.append(corners2)
    else:
        print('no chessboard corners found:' + str(img_id))
    #i += 1
# calibrate camera
# extract camera params from the json file
with open(camera_param_path, 'r') as f:
    data = json.load(f)

mtx_initial = data['camera_mtx_cam1']
# convert to numpy array
mtx_initial = np.array([[mtx_initial[0][0], 0, mtx_initial[0][2]], [0, mtx_initial[1][1], mtx_initial[1][2]], [0, 0, 1]])
dist_initial = data['distortion_coeffs_cam1']
dist_initial = np.array(dist_initial)
#params = [1834.486266291348, 1847.3890358463243, 1007.5551878599326, 719.0925998795733]
#params = [1841.9129156705603, 1845.9591700061214, 930.8966995563184, 697.18190504#26024]
#mtx_initial = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
#dist_initial = np.array([-0.1686216447107, 0.0381671597, 0.1195, -0.00077342, 0.0003974])
#dist_initial = np.array([-0.15240650306881118, 0.05910497555065293, -0.004109817709626536, -0.004845547797613459])
# use flag
flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
         cv2.CALIB_FIX_PRINCIPAL_POINT +
         cv2.CALIB_FIX_FOCAL_LENGTH +
         cv2.CALIB_ZERO_TANGENT_DIST +
         cv2.CALIB_FIX_K1 + cv2.CALIB_FIX_K2 +
         cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K4 +
         cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6)
ret, mtx, dist, r, t = cv2.calibrateCamera(objpoints, imgpoints, image_size, mtx_initial, dist_initial, flags=flags)

'''
R_tar2cam = []
t_tar2cam = []
for im_id in range(num_of_images):
    img_path = os.path.join(images_path, str(im_id) + '.png')
    img = cv2.imread(img_path)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, checker_board_size , None)
    if ret == True:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # Find the rotation and translation vectors.
        # rvecs=[rx, ry, rz] in rad, tvecs=[x, y, z] in m (defined by size_of_chessboard_squares_m)
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
        # target2cam
        R_tar2cam.append(cv2.Rodrigues(rvecs)[0])  # rotation vector to rotation matrix
        t_tar2cam.append(tvecs)
    else:
        print('no chessboard corners found:' + im_id)

print('got target2cam')
'''
# convert rvecs to 3x3 rotation matrix
rvecs = [cv2.Rodrigues(rvec)[0] for rvec in r]

# print(mtx, 'dist', dist)
# convert tvecs from pixels to meters

# tvecs = np.array(t)  # * square_size_meter
# iterate over the length of tvecs and rvecs
R_cam_optical_2_target_vecs = rvecs
t_cam_optical_2_target_vecs = t

R_target_2_cam_optical_vecs = np.array(rvecs)
t_target_2_cam_optical_vecs = np.array(t)
'''
# get inverse of R_target_2_cam_optical
R_cam_optical_2_target_vecs = []
t_cam_optical_2_target_vecs = []
for i in range(len(R_target_2_cam_optical_vecs)):
    R_cam_optical_2_target_vecs.append(np.transpose(R_target_2_cam_optical_vecs[i]))
    t_cam_optical_2_target_vecs.append(-np.matmul(np.transpose(R_target_2_cam_optical_vecs[i]), t_target_2_cam_optical_vecs[i]))
R_target_2_cam_optical_vecs = np.array(R_target_2_cam_optical_vecs)
t_target_2_cam_optical_vecs = np.array(t_target_2_cam_optical_vecs)
'''
# read the json file
with open(json_path, 'r') as f:
    data = json.load(f)
R_base_2_vicon_cam_vecs = []
t_base_2_vicon_cam_vecs = []
R_vicon_cam_2_base_vecs = []
t_vicon_cam_2_base_vecs = []
for i in range(len(data)):
    translation = data[str(i)]['translation']
    rotation = data[str(i)]['rotation']
    R_base_2_vicon_cam_quaternion = np.array(rotation)
    # convert quaternion to rotation matrix using Scipy
    R_base_2_vicon_cam = R.from_quat(R_base_2_vicon_cam_quaternion).as_matrix()
    t_base_2_vicon_cam = np.array(translation)

    R_base_2_vicon_cam_vecs.append(R_base_2_vicon_cam)
    t_base_2_vicon_cam_vecs.append(t_base_2_vicon_cam)

    # get inverse of R_base_2_vicon_cam
    R_vicon_cam_2_base = np.transpose(R_base_2_vicon_cam)
    t_vicon_cam_2_base = -np.matmul(np.transpose(R_base_2_vicon_cam), t_base_2_vicon_cam)
    R_vicon_cam_2_base_vecs.append(R_vicon_cam_2_base)
    t_vicon_cam_2_base_vecs.append(t_vicon_cam_2_base)

#R_base_2_vicon_cam_vecs = np.array(R_base_2_vicon_cam_vecs)
#t_base_2_vicon_cam_vecs = np.array(t_base_2_vicon_cam_vecs)
#R_vicon_cam_2_base_vecs = np.array(R_vicon_cam_2_base_vecs)
#t_vicon_cam_2_base_vecs = np.array(t_vicon_cam_2_base_vecs)

# calculate the eye in hand calibration. Try with PARK, HORAUD and ANDREFF
R, t = cv2.calibrateHandEye(R_base_2_vicon_cam_vecs, t_base_2_vicon_cam_vecs, R_cam_optical_2_target_vecs, t_cam_optical_2_target_vecs, method=cv2.CALIB_HAND_EYE_ANDREFF)
# Compute the error of the hand-eye calibration
error = cv2.norm(np.matmul(R_base_2_vicon_cam_vecs[0], R) - np.matmul(R, R_cam_optical_2_target_vecs[0])) + cv2.norm(t_base_2_vicon_cam_vecs[0] + np.matmul(R, t_cam_optical_2_target_vecs[0]))
print('error = ', error)

# Compute residuals
rotation_residuals = []
translation_residuals = []

for i in range(len(R_base_2_vicon_cam_vecs)):
    # Compute rotation residual
    R_error = np.matmul(R_base_2_vicon_cam_vecs[i], R) - np.matmul(R, R_cam_optical_2_target_vecs[i])
    rotation_error_norm = np.linalg.norm(R_error)  # Frobenius norm for rotation difference

    # Compute translation residual
    t_error = t_base_2_vicon_cam_vecs[i] + np.matmul(R, t_cam_optical_2_target_vecs[i]) - t
    translation_error_norm = np.linalg.norm(t_error)  # Euclidean norm for translation difference

    # Store residuals
    rotation_residuals.append(rotation_error_norm)
    translation_residuals.append(translation_error_norm)

# Print residuals
print(f"Rotation residuals (norm): {rotation_residuals}")
print(f"Translation residuals (norm): {translation_residuals}")

# Compute overall mean residuals
mean_rotation_residual = np.mean(rotation_residuals)
mean_translation_residual = np.mean(translation_residuals)

# homogenous vicon cam to cam optical
H = np.hstack((R, t))
H = np.vstack((H, np.array([[0, 0, 0, 1]])))
print('H_viconCam_2_cam_optical = ')
print(H)
# append the H_viconCam_2_cam_optical to already existing json file with data, as a dictionary entry

with open('/home/eventcamera/RGB_Event_cam_system/Annotation/camera_params.json', 'r') as f:
    read_data = json.load(f)
    read_data['H_cam_sys_2_rgb'] = H.tolist()
    #read_data['H_cam_sys_2_right'] = H.tolist()

with open('/home/eventcamera/RGB_Event_cam_system/Annotation/camera_params.json', 'w') as f:
    json.dump(read_data, f, indent=2)

