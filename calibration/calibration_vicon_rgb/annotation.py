# Transform the coordinates in RGB camera frame to event camera frame
# RGB camera is cam0, Event camera 1 is cam1 and event camera 2 is cam2
import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R


data_path = '/home/eventcamera/data/vicon_data/'
json_path_camera = os.path.join(data_path, 'vicon_coordinates_camera.json')
json_path_object = os.path.join(data_path, 'object.json')
# with open(json_path_camera, 'r') as f:
#    data_camera = json.load(f)
with open(json_path_object, 'r') as f:
    data_object = json.load(f)
rgb_image_path = '/home/eventcamera/data/rgb/'
rgb_timestamp = os.listdir(rgb_image_path)
rgb_timestamp.sort()

with open(json_path_object, 'r') as file:
    loaded_array = json.load(file)
# extract only timestamp in a numpy array from dictionary loaded_array
timestamp_object = []
for i in range(len(loaded_array)):
    #print(i)
    timestamp_object.append(loaded_array[str(i)]['timestamp'])
timestamp_object = np.array(timestamp_object)


def find_closest_elements(A, B):
    result = {}

    for a in A:
        closest_b = min(B, key=lambda x: abs(x - a))
        result[a] = closest_b
        B.remove(closest_b)

    return result


def remove_extension_and_convert_to_int(arr):
    # Remove ".png" extension and convert to integers
    modified_arr = [int(file_name[:-4]) for file_name in arr if file_name.endswith('.png')]
    return modified_arr


rgb_timestamp = remove_extension_and_convert_to_int(rgb_timestamp)
# convert list of strings to list of integers
timestamp_object = list(map(int, timestamp_object))
result_dict = find_closest_elements(rgb_timestamp, timestamp_object) # Output in format (rgb_timestamp, timestamp_object)
vicon_coord = []


for key, value in result_dict.items():
    value = str(value)
    #get translation and rotation from loaded_array for the timetamp values corresponding to value in result_dict
    translations_for_timestamps = {
        timestamp: np.array(loaded_array[timestamp]["translation"]) if timestamp in loaded_array else None
        for timestamp in value
    }
    #vicon_coord.append(loaded_array[str(value)])
vicon_coord = np.array(vicon_coord)


# Transformation matrix obtained from eye in hand calibration
H_cam_vicon_2_cam_optical = np.array([
    [-9.12530750e-03, - 1.08164565e-02, 9.99899862e-01, 0.0204],
    [-9.99946116e-01, - 4.85033329e-03, - 9.17819830e-03, 0.04],
    [4.94912316e-03, - 9.99929737e-01, - 1.07716128e-02, -0.04],
    [0., 0., 0., 1.]
])

R_base_2_vicon_cam_vecs = []
t_base_2_vicon_cam_vecs = []
for i in range(len(data_camera)):
    translation = data_camera[str(i)]['translation']
    rotation = data_camera[str(i)]['rotation']
    R_base_2_vicon_cam_quaternion = np.array(rotation)
    # convert quaternion to rotation matrix using Scipy
    R_base_2_vicon_cam = R.from_quat(R_base_2_vicon_cam_quaternion).as_matrix()
    t_base_2_vicon_cam = np.array(translation)
    R_base_2_vicon_cam_vecs.append(R_base_2_vicon_cam)
    t_base_2_vicon_cam_vecs.append(t_base_2_vicon_cam)
R_base_2_vicon_cam_vecs = np.array(R_base_2_vicon_cam_vecs)
t_base_2_vicon_cam_vecs = np.array(t_base_2_vicon_cam_vecs)

R_base_2_vicon_object_vecs = []
t_base_2_vicon_object_vecs = []
for i in range(len(data_object)):
    translation = data_object[str(i)]['translation']
    rotation = data_object[str(i)]['rotation']
    R_base_2_vicon_object_quaternion = np.array(rotation)
    # convert quaternion to rotation matrix using Scipy
    R_base_2_vicon_object = R.from_quat(R_base_2_vicon_object_quaternion).as_matrix()
    t_base_2_vicon_object = np.array(translation)
    R_base_2_vicon_object_vecs.append(R_base_2_vicon_object)
    t_base_2_vicon_object_vecs.append(t_base_2_vicon_object)
R_base_2_vicon_object_vecs = np.array(R_base_2_vicon_object_vecs)
t_base_2_vicon_object_vecs = np.array(t_base_2_vicon_object_vecs)

# make homogeneous transformation matrix
H_base_2_cam_vicon = np.eye(4)
H_base_2_cam_vicon[:3, :3] = rotation
H_base_2_cam_vicon[:3, 3] = translation

# make homogeneous transformation matrix from base to camera optical frame
H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)

# invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
# H_cam_optical_2_base = np.eye(4)
# H_cam_optical_2_base[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
# H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])
t_cam_optical_2_base = np.transpose(H_base_2_cam_optical[:3, :3])

rgb_img_path = "/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam0/1704386799592976847.png"
event_cam_img = "/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam1/1704386800701017000.png"
quat_cam2_cam1 = [0.00026235, -0.00376717, -0.00030698, 0.99999282]
quat_cam1_cam0 = [-0.00916228, -0.04119687, 0.0005243, 0.9991089]
R_cam1_cam0 = R.from_quat(quat_cam1_cam0).as_matrix()

# Transformation matrix from camera 1 to camera 0
H_cam1_cam0 = np.eye(4)
H_cam1_cam0[:3, :3] = R_cam1_cam0
H_cam1_cam0[:3, 3] = [0.05834856, 0.00157235, 0.02935724]

# Transformation matrix from camera 2 to camera 1
H_cam2_cam1 = np.eye(4)
H_cam2_cam1[:3, :3] = R.from_quat(quat_cam2_cam1).as_matrix()
H_cam2_cam1[:3, 3] = [-0.10245441, 0.00011387, 0.00283878]

# project point (x,y,z) in cam0 coordinate to cam1 coordinate
point_cam0 = np.array([
    [1, 0, 0, t_cam_optical_2_base[0]],
    [0, 1, 0, t_cam_optical_2_base[1]],
    [0, 0, 1, t_cam_optical_2_base[2]],
    [0, 0, 0, 1]])
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
cv2.circle(event_img, (point_cam1[0, 3], 819), 10, (0, 0, 255), -1)
cv2.imshow("img", event_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
