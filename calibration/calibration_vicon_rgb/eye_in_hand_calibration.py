"""
Eye in hand calibration
this script that takes images of checkerboard and a robot pose and output camera calibration
"""

import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R

#############  Define Paths and Parameters #############
square_size_meter = 0.05
data_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/second_calib'
checker_board_size = (10, 7)
#params = [2592.7798180209766, 2597.1074116646814, 1121.2441077660412, 690.1066893999352]
# distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])
params = [1936.8622, 1933.3, 985.7, 771]
distortion_coefficients = np.array([-0.12905658, -0.01019267, 0.00562304, -0.00015354, 0.13542021])

json_path = os.path.join(data_path, 'vicon_coordinates.json')
camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])

images_path = os.path.join(data_path, 'images')
num_of_images = len(os.listdir(images_path))
###############

objp = np.zeros((checker_board_size[0] * checker_board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checker_board_size[0], 0:checker_board_size[1]].T.reshape(-1, 2)

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)

# iterate over images to get checkboard transformation
for img_id in range(num_of_images):
    img_path = os.path.join(images_path, str(img_id) + '.png')
    img = cv2.imread(img_path)

    # undistort image using camera matrix and distortion coefficients

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

# calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], camera_matrix,
                                                   distortion_coefficients)
# print(mtx, 'dist', dist)
# convert tvecs from pixels to meters using square size
tvecs = np.array(tvecs)*0.005

# iterate over the length of tvecs and rvecs
R_target_2_cam_optical_vecs = []
t_target_2_cam_optical_vecs = []
for i in range(len(tvecs)):
    # get rotation matrix from Rodrigues
    rvecs_cam_optical_2_target, _ = cv2.Rodrigues(rvecs[i])
    tvecs_cam_optical_2_target = np.array(tvecs[i])
    R_target_2_cam_optical = np.transpose(rvecs_cam_optical_2_target)
    t_target_2_cam_optical = -np.matmul(np.transpose(rvecs_cam_optical_2_target), tvecs_cam_optical_2_target)
    R_target_2_cam_optical_vecs.append(R_target_2_cam_optical)
    t_target_2_cam_optical_vecs.append(t_target_2_cam_optical)

# get transformation from base to gripper
# read the json file
with open(json_path, 'r') as f:
    data = json.load(f)
R_vicon_cam_2_base_vecs = []
t_vicon_cam_2_base_vecs = []
for i in range(len(data)):
    translation = data[str(i)]['translation']
    rotation = data[str(i)]['rotation']
    R_base_2_vicon_cam_quaternion = np.array(rotation)
    R_base_2_vicon_cam_quaternion = np.array([R_base_2_vicon_cam_quaternion[3], R_base_2_vicon_cam_quaternion[0],
                                             R_base_2_vicon_cam_quaternion[1], R_base_2_vicon_cam_quaternion[2]])
    # convert quaternion to rotation matrix using Scipy
    R_base_2_vicon_cam = R.from_quat(R_base_2_vicon_cam_quaternion).as_matrix()

    t_base_2_vicon_cam = np.array(translation)

    R_vicon_cam_2_base = np.transpose(R_base_2_vicon_cam)
    t_vicon_cam_2_base = -np.matmul(np.transpose(R_base_2_vicon_cam), t_base_2_vicon_cam)
    R_vicon_cam_2_base_vecs.append(R_vicon_cam_2_base)
    t_vicon_cam_2_base_vecs.append(t_vicon_cam_2_base)
R_vicon_cam_2_base_vecs = np.array(R_vicon_cam_2_base_vecs)
t_vicon_cam_2_base_vecs = np.array(t_vicon_cam_2_base_vecs)

# calculate the eye in hand calibration
R_cam_optical_2_cam_vicon, t_cam_optical_2_cam_vicon = cv2.calibrateHandEye(R_vicon_cam_2_base_vecs,
                                                                            t_vicon_cam_2_base_vecs,
                                                                            R_target_2_cam_optical_vecs,
                                                                            t_target_2_cam_optical_vecs,
                                                                            cv2.CALIB_HAND_EYE_TSAI)
#print('R_g_c: ', R_g_c)
#print('t_g_c: ', t_g_c)

# compute R_c_g and t_c_g
R_c_g = np.transpose(R_cam_optical_2_cam_vicon)
t_c_g = -np.matmul(np.transpose(R_cam_optical_2_cam_vicon), t_cam_optical_2_cam_vicon)

# homogenous gripper to camera
H = np.concatenate((R_c_g, t_c_g), axis=1)
H = np.concatenate((H, np.array([[0, 0, 0, 1]])), axis=0)
print('H_viconCam_2_camOptical:')
print(H)


