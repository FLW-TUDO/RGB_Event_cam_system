# Transform the coordinates in RGB camera frame to event camera frame
# RGB camera is cam0, Event camera 1 is cam1 and event camera 2 is cam2
import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import trimesh
import open3d as o3d
import shutil
from utilities import *

# 1: "wooden_pallet", 2: "small_klt", 3: "big_klt", 4: "blue_klt", 5: "shogun_box",
# 6: "kronen_bier_crate", 7: "brinkhoff_bier_crate", 8: "zivid_cardboard_box", 9: "dell_carboard_box", 10: "ciatronic_carboard_box"

object_id = 1
# import object data from json file
with open('/home/eventcamera/RGB_Event_cam_system/Annotation/Annotation_rgb_ec/obj_model/models_info.json', 'r') as file:
    obj_model_data = json.load(file)
object_len_x = obj_model_data[str(object_id)]['size_x']
object_len_y = obj_model_data[str(object_id)]['size_y']
object_len_z = obj_model_data[str(object_id)]['size_z']

path = '/home/eventcamera/data/dataset/pallet_4/'
json_path_camera_sys = '/home/eventcamera/data/dataset/pallet_4/vicon_data/event_cam_sys.json'
json_path_object = '/home/eventcamera/data/dataset/pallet_4/vicon_data/object.json'
path_event_cam_left_img = '/home/eventcamera/data/dataset/pallet_4/event_cam_left/e2calib/'
path_event_cam_right_img = '/home/eventcamera/data/dataset/pallet_4/event_cam_right/e2calib/'
output_dir = '/home/eventcamera/data/dataset/pallet_4/annotation/'
rgb_image_path = '/home/eventcamera/data/dataset/pallet_4/rgb/'
obj_path = '/home/eventcamera/RGB_Event_cam_system/Annotation/Annotation_rgb_ec/obj_model/obj_' + str(object_id) + '.ply'

# if path does not exist, create the path
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

for filename in os.listdir(output_dir):
        file_path = os.path.join(output_dir, filename)
        os.unlink(file_path)


# with open(json_path_camera, 'r') as f:
#    data_camera = json.load(f)

rgb_timestamp = os.listdir(rgb_image_path)
rgb_timestamp.sort()

event_cam_left_timestamp = os.listdir(path_event_cam_left_img)
event_cam_left_timestamp.sort()

event_cam_right_timestamp = os.listdir(path_event_cam_right_img)
event_cam_right_timestamp.sort()
H_cam_optical_2_base = np.eye(4)

with open(json_path_object, 'r') as file:
    object_array = json.load(file)
# extract only timestamp in a numpy array from dictionary loaded_array
timestamp_object = []

for k, v in object_array.items():
    timestamp_object.append(v['timestamp'])
timestamp_object = np.array(timestamp_object)

rgb_timestamp = remove_extension_and_convert_to_int(rgb_timestamp)
event_cam_left_timestamp = remove_extension_and_convert_to_int(event_cam_left_timestamp)
event_cam_right_timestamp = remove_extension_and_convert_to_int(event_cam_right_timestamp)
# convert list of strings to list of integers
# Associate timestamps in both event cameras to rgb camera timestamps.
timestamp_object = list(map(int, timestamp_object))
result_dict = find_closest_elements(rgb_timestamp,
                                    timestamp_object)  # Output in format (rgb_timestamp, timestamp_object)
#vicon_coord = []
timestamps_closest_object = list(result_dict.values())
result_dict_left = find_closest_elements(rgb_timestamp, event_cam_left_timestamp)
result_dict_right = find_closest_elements(rgb_timestamp, event_cam_right_timestamp)

result_dict_left = remove_delayed_timestamps(result_dict_left)
result_dict_right = remove_delayed_timestamps(result_dict_right)
result_dict = remove_delayed_timestamps(result_dict)

timestamp_closest_ec_left = list(result_dict_left.values())
timestamp_closest_ec_right = list(result_dict_right.values())

translations_with_timestamps = {
    timestamp: np.array(object_array[str(timestamp)]["translation"])
    for timestamp in timestamps_closest_object}
rotations_with_timestamps = {
    timestamp: np.array(object_array[str(timestamp)]["rotation"])
    for timestamp in timestamps_closest_object
}

# load camera parameters and transformation data from json file
with open('/home/eventcamera/RGB_Event_cam_system/Annotation/camera_params.json', 'r') as file:
    data = json.load(file)

camera_matrix = np.array(data['camera_matrix'])
distortion_coefficients = np.array(data['distortion_coefficients'])
camera_mtx_cam2 = np.array(data['camera_mtx_cam2'])
distortion_coeffs_cam2 = np.array(data['distortion_coeffs_cam2'])
camera_mtx_cam1 = np.array(data['camera_mtx_cam1'])
distortion_coeffs_cam1 = np.array(data['distortion_coeffs_cam1'])
H_cam_vicon_2_cam_optical = np.array(data['H_cam_vicon_2_cam_optical'])
H_cam1_2_rgb = np.array(data['H_cam1_2_rgb'])
H_cam2_cam1 = np.array(data['H_cam2_cam1'])

######### Here we save the transformations for the dataset. Annotations are not happening here. #########
# Read the vicon coordinates of the even camera system. Traverse through the coordinates
with open(json_path_camera_sys, 'r') as f:
    data = json.load(f)
save_transformations(data, H_cam_vicon_2_cam_optical, object_array, H_cam1_2_rgb, H_cam2_cam1, path)

################## ANNOTATIONS #################
count = 0
with open(path + 'transformations.json', 'r') as file:
    projected_point_rgb_ec1_ec2 = json.load(file)

timestamp_closest_ec_right = sorted(timestamp_closest_ec_right)
timestamp_closest_ec_left = sorted(timestamp_closest_ec_left)
rgb_timestamp = sorted(rgb_timestamp)

for (kr, vr), (k, v) in zip(rotations_with_timestamps.items(), translations_with_timestamps.items()):
    # kr and k are timestamps for respective values
    print(kr)
    ############# Defining paths for images #############
    rgb_t = rgb_timestamp[count]
    ec_left = timestamp_closest_ec_left[count]
    ec_right = timestamp_closest_ec_right[count]
    rgb_img_path = rgb_image_path + str(rgb_t) + ".png"
    event_cam_left = path_event_cam_left_img + str(ec_left) + ".png"
    event_cam_right = path_event_cam_right_img + str(ec_right) + ".png"
    #####################################################

    t_cam_optical_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['t_cam_optical_2_point'])
    H_cam_optical_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['H_cam_optical_2_point'])
    rotation = H_cam_optical_2_point[:3, :3]
    print(t_cam_optical_2_point)

    ####### Import object ply file and create a mesh for visualization #######
    obj_geometry = trimesh.load_mesh(obj_path)
    if not isinstance(obj_geometry, trimesh.Trimesh):
        print("The object is not a Trimesh object. It is a", type(obj_geometry))
    trimesh_object = obj_geometry.convex_hull
    points_3d = np.array(trimesh_object.sample(50000)) / 1000
    vertices = np.array(trimesh_object.vertices) / 1000

    # translate the object to match the center with the vicon camera frame
    vertices, points_3d = get_translated_points_vertice(object_id, vertices, points_3d)
    '''
    #################################### Exporting bounding box and pose values to a json file ####################################
    save_bbox_values(output_dir, object_3d_transform_points)
    # Save pose values to a json file
    
    rotmat = R.from_matrix(rotation)
    euler_angles = rotmat.as_euler('xyz', degrees=True)
    pose = np.concatenate((center_3d, euler_angles))

    file = os.path.join(output_dir, "pose.json")
    with open(file, 'a') as json_file:
        json.dump(pose.tolist(), json_file)
        json_file.write('\n')
    '''
    ############ RGB Image ############
    img_rgb = project_points_to_image_plane(t_cam_optical_2_point, rotation, rgb_img_path, points_3d, vertices,
                                                    camera_matrix, distortion_coefficients)

    ############ Event camera 1 ############

    t_cam1_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['point_event_cam_left'])
    img_event_cam_1 = project_points_to_image_plane(t_cam1_2_point, rotation, event_cam_left, points_3d, vertices,
                                                    camera_mtx_cam1, distortion_coeffs_cam1)

    ############ Event camera 2 ############
    t_cam2_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['point_event_cam_right'])
    img_event_cam_2 = project_points_to_image_plane(t_cam2_2_point, rotation, event_cam_right, points_3d, vertices,
                                                    camera_mtx_cam2, distortion_coeffs_cam2)

    ########### Display the images ###########
    img_rgb = cv2.resize(img_rgb, (568, 426))
    img_event_cam_1 = cv2.resize(img_event_cam_1, (568, 426))
    img_event_cam_2 = cv2.resize(img_event_cam_2, (568, 426))

    concatenated_images = np.hstack((img_rgb, img_event_cam_1, img_event_cam_2))
    output_path = os.path.join(output_dir, f'image_{count:03d}.jpg')
    cv2.imwrite(output_path, concatenated_images)

    cv2.waitKey(0)
    count += 1
    cv2.destroyAllWindows()
