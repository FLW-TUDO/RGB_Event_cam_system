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
root = '/media/eventcamera/event_data/calibration/Extrinsic/third_calib/'
images_path = '/media/eventcamera/event_data/calibration/Extrinsic/third_calib/e2calib_25/'

square_size_meter = 0.125
image_size = (640, 480)
start_t = 1739702735004679844
end_t = 1739703524177583024
total_time = int((end_t - start_t) / 1000000000)
# store the timestamp values in a list. The time should increment 3 sec from start to end t
timestamp_list = [start_t + i * 9000000000 for i in range(int(total_time/9))]
# =============================================================================
def find_closest_elements(A, B):
    result = {}

    for a in A:
        closest_b = min(B, key=lambda x: abs(x - a))
        result[a] = closest_b
        B.remove(closest_b)

    return result

with open(root + '/event_cam_sys.json', 'r') as f:
    data_cam_sys = json.load(f)
# the keys are the timestamps. Extract timestamps to a list. Convert all timestamp values to int
timestamps_cam_sys = list(data_cam_sys.keys())
timestamps_cam_sys = [int(t) for t in timestamps_cam_sys]
closest_cam_sys = find_closest_elements(timestamp_list, timestamps_cam_sys)
# store only te values in a list
closest_cam_sys = list(closest_cam_sys.values())

event_files = [f for f in os.listdir(images_path) if f.endswith('.png')]
event_timestamps = sorted([int(f[:-4]) for f in event_files])
event_closest_timestamp = find_closest_elements(timestamp_list, event_timestamps)
event_closest_timestamp = list(event_closest_timestamp.values())


camera_param_path = '/home/eventcamera/RGB_Event_cam_system/Annotation/camera_params.json'
checker_board_size = (8, 5)
#params = [1860.9939429743338, 1862.40872909852, 917.1768824325476, 679.5399523739977]
#distortion_coefficients = np.array([-0.07869357, 0.02253124, 0.00171336, 0.00272475])
# =============================================================================

num_of_images = len(event_closest_timestamp)
###############

objp = np.zeros((checker_board_size[0] * checker_board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checker_board_size[0], 0:checker_board_size[1]].T.reshape(-1, 2)
objp = objp * square_size_meter

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
pop_list = []
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
i = 0
# iterate over images to get checkerboard transformation
for img_id in range(num_of_images):
    img_path = os.path.join(images_path, str(event_closest_timestamp[i]) + '.png')
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
        # remove the timestamp entry from closest_cam_sys. Shift other values to the left
        pop_list.append(i)

    i += 1
# pop all the elements in pop_list from closest_cam_sys
for i in sorted(pop_list, reverse=True):  # Sort in descending order
        if 0 <= i < len(closest_cam_sys):  # Ensure index is valid
            closest_cam_sys.pop(i)
#read camera param path
with open(camera_param_path, 'r') as f:
    data_param = json.load(f)
mtx_initial = data_param['camera_mtx_cam2']
# convert to numpy array
mtx_initial = np.array([[mtx_initial[0][0], 0, mtx_initial[0][2]], [0, mtx_initial[1][1], mtx_initial[1][2]], [0, 0, 1]])
dist_initial = data_param['distortion_coeffs_cam2']
dist_initial = np.array(dist_initial)

# use flag
flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
         cv2.CALIB_FIX_PRINCIPAL_POINT +
         cv2.CALIB_FIX_FOCAL_LENGTH +
         cv2.CALIB_ZERO_TANGENT_DIST +
         cv2.CALIB_FIX_K1 + cv2.CALIB_FIX_K2 +
         cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K4 +
         cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6)
ret, mtx, dist, r, t = cv2.calibrateCamera(objpoints, imgpoints, image_size, mtx_initial, dist_initial, flags=flags)

# convert rvecs to 3x3 rotation matrix
rvecs = [cv2.Rodrigues(rvec)[0] for rvec in r]

R_cam_optical_2_target_vecs = rvecs
t_cam_optical_2_target_vecs = t

R_target_2_cam_optical_vecs = np.array(rvecs)
t_target_2_cam_optical_vecs = np.array(t)

# read the json file

R_base_2_vicon_cam_vecs = []
t_base_2_vicon_cam_vecs = []

for i in closest_cam_sys:
    translation = data_cam_sys[str(i)]['translation']
    rotation = data_cam_sys[str(i)]['rotation']
    R_base_2_vicon_cam_quaternion = np.array(rotation)
    # convert quaternion to rotation matrix using Scipy
    R_base_2_vicon_cam = R.from_quat(R_base_2_vicon_cam_quaternion).as_matrix()
    t_base_2_vicon_cam = np.array(translation)

    R_base_2_vicon_cam_vecs.append(R_base_2_vicon_cam)
    t_base_2_vicon_cam_vecs.append(t_base_2_vicon_cam)

# calculate the eye in hand calibration. Try with PARK, HORAUD and ANDREFF
R, t = cv2.calibrateHandEye(R_base_2_vicon_cam_vecs, t_base_2_vicon_cam_vecs, R_cam_optical_2_target_vecs, t_cam_optical_2_target_vecs, method=cv2.CALIB_HAND_EYE_ANDREFF)

# homogenous vicon cam to cam optical
H = np.hstack((R, t))
H = np.vstack((H, np.array([[0, 0, 0, 1]])))
print('H_viconCam_2_cam_optical = ')
print(H)
# append the H_viconCam_2_cam_optical to already existing json file with data, as a dictionary entry

with open('/Annotation/camera_params.json', 'r') as f:
    read_data = json.load(f)
    #read_data['H_cam_sys_2_rgb'] = H.tolist()
    read_data['H_cam_sys_2_right'] = H.tolist()

with open('/Annotation/camera_params.json', 'w') as f:
    json.dump(read_data, f, indent=2)

