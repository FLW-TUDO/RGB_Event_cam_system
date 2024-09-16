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

object_id = 8
# import object data from json file
with open('/home/eventcamera/RGB_Event_cam_system/Annotation/Annotation_rgb_ec/obj_model/models_info.json', 'r') as file:
    obj_model_data = json.load(file)
object_len_x = obj_model_data[str(object_id)]['size_x']
object_len_y = obj_model_data[str(object_id)]['size_y']
object_len_z = obj_model_data[str(object_id)]['size_z']

json_path_camera_sys = '/home/eventcamera/data/dataset/aug16/zivid_1/vicon_data/event_cam_sys.json'
json_path_object = '/home/eventcamera/data/dataset/aug16/zivid_1/vicon_data/object1.json'
path_event_cam_left_img = '/home/eventcamera/data/dataset/aug16/zivid_1/event_cam_left/e2calib/'
path_event_cam_right_img = '/home/eventcamera/data/dataset/aug16/zivid_1/event_cam_right/e2calib/'
output_dir = '/home/eventcamera/data/dataset/aug16/zivid_1/annotation/'
rgb_image_path = '/home/eventcamera/data/dataset/aug16/zivid_1/rgb/'
obj_path = '/home/eventcamera/data/KLT/zivid.ply'

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
with open('/home/eventcamera/data/transformations/camera_params.json', 'r') as file:
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
save_transformations(data, H_cam_vicon_2_cam_optical, object_array, H_cam1_2_rgb, H_cam2_cam1)

################## ANNOTATIONS #################
count = 0
with open('/home/eventcamera/data/transformations/transformations.json', 'r') as file:
    projected_point_rgb_ec1_ec2 = json.load(file)

timestamp_closest_ec_right = sorted(timestamp_closest_ec_right)
timestamp_closest_ec_left = sorted(timestamp_closest_ec_left)
rgb_timestamp = sorted(rgb_timestamp)

for (kr, vr), (k, v) in zip(rotations_with_timestamps.items(), translations_with_timestamps.items()):
    print(kr)
    rgb_t = rgb_timestamp[count]
    ec_left = timestamp_closest_ec_left[count]
    ec_right = timestamp_closest_ec_right[count]
    rgb_img_path = rgb_image_path + str(rgb_t) + ".png"
    event_cam_left = path_event_cam_left_img + str(ec_left) + ".png"
    event_cam_right = path_event_cam_right_img + str(ec_right) + ".png"

    t_cam_optical_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['t_cam_optical_2_point'])
    H_cam_optical_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['H_cam_optical_2_point'])

    print(t_cam_optical_2_point)
    points_2d = cv2.projectPoints(t_cam_optical_2_point, np.eye(3), np.zeros(3), camera_matrix, distortion_coefficients)
    points_2d = np.round(points_2d[0]).astype(int)
    img_test = cv2.imread(rgb_img_path)
    img_test = cv2.circle(img_test, tuple(points_2d[0][0]), 20, (255, 0, 0), -1)

    ####### Import object ply file and create a mesh for visualization #######
    obj_geometry = trimesh.load_mesh(obj_path)
    if not isinstance(obj_geometry, trimesh.Trimesh):
        print("The object is not a Trimesh object. It is a", type(obj_geometry))
    trimesh_object = obj_geometry.convex_hull
    points_3d = np.array(trimesh_object.sample(3000)) / 1000
    vertices = np.array(trimesh_object.vertices) / 1000

    # translate the object to match the center with the vicon camera frame
    vertices, points_3d = get_translated_points_vertice(object_id, vertices, points_3d)

    object_3d_transform_points = np.matmul(H_cam_optical_2_point, np.vstack((points_3d.T, np.ones(points_3d.shape[0]))))[:3, :].T
    object_3d_transform_vertices = np.matmul(H_cam_optical_2_point, np.vstack((vertices.T, np.ones(vertices.shape[0]))))[:3, :].T
    center_3d = np.mean(object_3d_transform_points, axis=0)

    #################################### Exporting bounding box and pose values to a json file ####################################
    save_bbox_values(output_dir, object_3d_transform_points)
    # Save pose values to a json file
    rotation = H_cam_optical_2_point[:3, :3]
    rotmat = R.from_matrix(rotation)
    euler_angles = rotmat.as_euler('xyz', degrees=True)
    pose = np.concatenate((center_3d, euler_angles))

    file = os.path.join(output_dir, "pose.json")
    with open(file, 'a') as json_file:
        json.dump(pose.tolist(), json_file)
        json_file.write('\n')
    ##############################################################################################################################

    klt_2d_points, _ = cv2.projectPoints(object_3d_transform_points, np.eye(3), np.zeros(3), camera_matrix,
                                         distortion_coefficients)
    klt_2d_vertices, _ = cv2.projectPoints(object_3d_transform_vertices, np.eye(3), np.zeros(3), camera_matrix,
                                         distortion_coefficients)
    center_2d, _ = cv2.projectPoints(np.array([center_3d]), np.eye(3), np.zeros(3), camera_matrix,
                                     distortion_coefficients)
    center_2d = center_2d[0, 0]

    for point in klt_2d_points:
        img_test = cv2.circle(img_test, tuple(point[0].astype(int)), 2, (200, 200, 200), -1)
    for point in klt_2d_vertices:
        img_test = cv2.circle(img_test, tuple(point[0].astype(int)), 8, (0, 0, 255), -1)
    img_test = cv2.circle(img_test, tuple(center_2d.astype(int)), 8, (0, 0, 255), -1)

    ############ Event camera 1 ############
    t_cam1_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['point_event_cam_left'])
    H_cam1_2_point = np.eye(4)
    H_cam1_2_point[:3, :3] = rotation
    H_cam1_2_point[:3, 3] = t_cam1_2_point
    points_2d_cam1 = cv2.projectPoints(np.array([t_cam1_2_point]), np.eye(3), np.zeros(3), camera_mtx_cam1,
                                       distortion_coeffs_cam1)
    points_2d_cam1 = np.round(points_2d_cam1[0]).astype(int)
    print(points_2d_cam1)
    # Display the 2d points on the image
    img_test_cam1 = cv2.imread(event_cam_left)
    img_test1 = cv2.circle(img_test_cam1, tuple(points_2d_cam1[0][0]), 5, (255, 0, 0), -1)

    klt_3d_transform_points = np.matmul(H_cam1_2_point, np.vstack((points_3d.T, np.ones(points_3d.shape[0]))))[
                              :3, :].T
    klt_3d_transform_vertices = np.matmul(H_cam1_2_point, np.vstack((vertices.T, np.ones(vertices.shape[0]))))[
                                :3, :].T

    klt_2d_points, _ = cv2.projectPoints(klt_3d_transform_points, np.eye(3), np.zeros(3), camera_mtx_cam1,
                                         distortion_coeffs_cam1)
    klt_2d_vertices, _ = cv2.projectPoints(klt_3d_transform_vertices, np.eye(3), np.zeros(3), camera_mtx_cam1,
                                           distortion_coeffs_cam1)

    for point in klt_2d_points:
        img_test1 = cv2.circle(img_test1, tuple(point[0].astype(int)), 1, (255, 255, 255), -1)
    for point in klt_2d_vertices:
        img_test1 = cv2.circle(img_test1, tuple(point[0].astype(int)), 3, (0, 0, 255), -1)

    ############ Event camera 2 ############
    t_cam2_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['point_event_cam_right'])
    H_cam2_2_point = np.eye(4)
    H_cam2_2_point[:3, :3] = rotation
    H_cam2_2_point[:3, 3] = t_cam2_2_point

    points_2d_cam2 = cv2.projectPoints(np.array([t_cam2_2_point]), np.eye(3), np.zeros(3), camera_mtx_cam2,
                                       distortion_coeffs_cam2)
    points_2d_cam2 = np.round(points_2d_cam2[0]).astype(int)
    print(points_2d_cam2)
    img_test_cam2 = cv2.imread(event_cam_right)
    # Display the 2d points on the image
    img_test2 = cv2.circle(img_test_cam2, tuple(points_2d_cam2[0][0]), 5, (255, 0, 0), -1)

    klt_3d_transform_points = np.matmul(H_cam2_2_point, np.vstack((points_3d.T, np.ones(points_3d.shape[0]))))[
                              :3, :].T
    klt_3d_transform_vertices = np.matmul(H_cam2_2_point, np.vstack((vertices.T, np.ones(vertices.shape[0]))))[
                                :3, :].T

    klt_2d_points, _ = cv2.projectPoints(klt_3d_transform_points, np.eye(3), np.zeros(3), camera_mtx_cam2,
                                         distortion_coeffs_cam2)
    klt_2d_vertices, _ = cv2.projectPoints(klt_3d_transform_vertices, np.eye(3), np.zeros(3), camera_mtx_cam2,
                                           distortion_coeffs_cam2)

    for point in klt_2d_points:
        img_test2 = cv2.circle(img_test2, tuple(point[0].astype(int)), 1, (255, 255, 255), -1)
    for point in klt_2d_vertices:
        img_test2 = cv2.circle(img_test2, tuple(point[0].astype(int)), 3, (0, 0, 255), -1)

    ########### Display the images ###########
    img_test = cv2.resize(img_test, (568, 426))
    img_test1 = cv2.resize(img_test1, (568, 426))
    img_test2 = cv2.resize(img_test2, (568, 426))

    concatenated_images = np.hstack((img_test, img_test1, img_test2))
    output_path = os.path.join(output_dir, f'image_{count:03d}.jpg')
    cv2.imwrite(output_path, concatenated_images)

    cv2.waitKey(0)
    count += 1
    cv2.destroyAllWindows()
