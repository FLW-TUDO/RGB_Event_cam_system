import numpy as np
from scipy.spatial.transform import Rotation as R
import json
import os
import cv2

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

def remove_delayed_timestamps(result_dict, threshold):
    keys_to_remove = []
    for key, value in result_dict.items():
        deviation = abs(int(key) - int(value))

        if deviation > threshold:
            keys_to_remove.append(key)

    for key in keys_to_remove:
        del result_dict[key]
    return result_dict

def save_transformations(vicon_data_camera_sys, H_cam_vicon_2_rgb, vicon_object_data, H_cam1_2_rgb, H_cam2_cam1, path):
    transformations = {}
    for i, v in vicon_data_camera_sys.items():
        if i == str(len(vicon_data_camera_sys) - 1):
            continue
        vicon_cam_translation = vicon_data_camera_sys[str(i)]['translation']
        vicon_cam_rotation_quat = vicon_data_camera_sys[str(i)]['rotation']

        # get rotation matrix from quaternion
        rotation_vicon_cam = R.from_quat(vicon_cam_rotation_quat).as_matrix()
        # make homogeneous transformation matrix
        H_world_2_cam_vicon = np.eye(4)
        H_world_2_cam_vicon[:3, :3] = rotation_vicon_cam
        H_world_2_cam_vicon[:3, 3] = vicon_cam_translation

        # make homogeneous transformation matrix from vicon to camera optical frame
        H_world_2_rgb = np.matmul(H_world_2_cam_vicon, H_cam_vicon_2_rgb)

        # invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
        H_rgb_2_world = np.eye(4)
        H_rgb_2_world[:3, :3] = np.transpose(H_world_2_rgb[:3, :3])
        H_rgb_2_world[:3, 3] = -np.matmul(np.transpose(H_world_2_rgb[:3, :3]),
                                                 H_world_2_rgb[:3, 3])
        t_x = vicon_object_data[str(v['timestamp'])]['translation'][0]
        t_y = vicon_object_data[str(v['timestamp'])]['translation'][1]
        t_z = vicon_object_data[str(v['timestamp'])]['translation'][2]
        r_x = vicon_object_data[str(v['timestamp'])]['rotation'][0]
        r_y = vicon_object_data[str(v['timestamp'])]['rotation'][1]
        r_z = vicon_object_data[str(v['timestamp'])]['rotation'][2]
        r_w = vicon_object_data[str(v['timestamp'])]['rotation'][3]
        rotation = R.from_quat([r_x, r_y, r_z, r_w]).as_matrix()
        # world_2_object are the recorded vicon values for the object
        H_world_2_object = np.eye(4)
        H_world_2_object[:3, :3] = rotation
        H_world_2_object[:3, 3] = [t_x, t_y, t_z]

        H_rgb_2_object = np.matmul(H_rgb_2_world, H_world_2_object)
        t_rgb_2_object = H_rgb_2_object[:3, 3]

        # project object (x,y,z) in rgb coordinate to cam1 coordinate
        H_cam1_2_object = np.matmul(H_cam1_2_rgb, H_rgb_2_object)
        t_cam1_2_object = H_cam1_2_object[:3, 3]
        H_cam2_2_object = np.matmul(H_cam2_cam1, H_cam1_2_object)
        t_cam2_2_object = H_cam2_2_object[:3, 3]
        transformations[str(vicon_data_camera_sys[str(i)]['timestamp'])] = {'H_rgb_2_vicon': H_rgb_2_world.tolist(),
                                                           'H_rgb_2_object': H_rgb_2_object.tolist(),
                                                           'H_world_2_cam_vicon': H_world_2_cam_vicon.tolist(),
                                                           't_rgb_2_object': t_rgb_2_object.tolist(),
                                                           'H_cam1_2_object': H_cam1_2_object.tolist(),
                                                           'H_cam2_2_object': H_cam2_2_object.tolist(),
                                                           'rotation': rotation.tolist(),
                                                           'timestamp': str(v['timestamp'])
                                                           }
    with open(path + 'transformations.json', 'w') as json_file:
        json.dump(transformations, json_file, indent=2)
    print('saved transformations data')


def get_translated_points_vertice(object_id, vertices, points_3d, object_len_z):
    if object_id == 1:
        translation_vector = np.array([0, 0, -0.072])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 2:
        translation_vector = np.array([0.0, 0, 0.072])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 3:
        translation_vector = np.array([-0.05, 0, 0])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 4:
        translation_vector = np.array([0, 0, 0])
        vertices -= translation_vector
        points_3d -= translation_vector
        rotation_matrix = R.from_euler('z', 90, degrees=True).as_matrix()
        vertices = np.dot(vertices, rotation_matrix)
        points_3d = np.dot(points_3d, rotation_matrix)

    if object_id == 6:
        translation_vector = np.array([0, 0, -object_len_z/2000])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 8:
        translation_vector = np.array([0, 0, 0.08])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 9:
        rotation_matrix = R.from_euler('z', 90, degrees=True).as_matrix()
        vertices = np.dot(vertices, rotation_matrix)
        points_3d = np.dot(points_3d, rotation_matrix)
        translation_vector = np.array([0, 0, 0.0])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 10:
        translation_vector = np.array([0, 0.0, 0])
        vertices -= translation_vector
        points_3d -= translation_vector

    return vertices, points_3d

def save_bbox_values(output_dir, object_3d_transform_points):
    #################################### Exporting bounding box and pose values to a json file ####################################

    # compute xmin, xmax, ymin, ymax, zmin, zmax
    xmin = np.min(object_3d_transform_points[:, 0])
    xmax = np.max(object_3d_transform_points[:, 0])
    ymin = np.min(object_3d_transform_points[:, 1])
    ymax = np.max(object_3d_transform_points[:, 1])
    zmin = np.min(object_3d_transform_points[:, 2])
    zmax = np.max(object_3d_transform_points[:, 2])
    # Save above values in an array
    Bbox = np.array([xmin, xmax, ymin, ymax, zmin, zmax])

    # append the Bbox values to a json file row wise
    file = os.path.join(output_dir, "bounding_box_labels.json")
    with open(file, 'a') as json_file:
        json.dump(Bbox.tolist(), json_file)
        json_file.write('\n')

def project_points_to_image_plane(H_cam_2_object, img_path, points_3d, vertices,
                                  camera_matrix, distortion_coefficients, output_dir, save = False):
    #points_2d_cam1 = cv2.projectPoints(np.array([t_cam_2_point]), np.eye(3), np.zeros(3), camera_matrix,
     #                                  distortion_coefficients)
    #points_2d_cam1 = np.round(points_2d_cam1[0]).astype(int)
    img_temp_cam = cv2.imread(img_path)
    # rectify the image with the intrinsic parameters
    img_temp_cam = cv2.undistort(img_temp_cam, camera_matrix, distortion_coefficients)

    #img_temp_cam = cv2.circle(img_temp_cam, tuple(points_2d_cam1[0][0]), 5, (255, 0, 0), -1)
    object_3d_transform_points = np.matmul(H_cam_2_object, np.vstack((points_3d.T, np.ones(points_3d.shape[0]))))[
                                 :3, :].T
    object_3d_transform_vertices = np.matmul(H_cam_2_object, np.vstack((vertices.T, np.ones(vertices.shape[0]))))[
                                   :3, :].T
    center_3d = np.mean(object_3d_transform_points, axis=0)
    if save:
        save_bbox_values(output_dir, object_3d_transform_points)
        save_pose(H_cam_2_object, center_3d, output_dir)

    # project 3d points to image plane
    object_2d_points, _ = cv2.projectPoints(object_3d_transform_points, np.eye(3), np.zeros(3), camera_matrix,
                                         distortion_coefficients)
    object_2d_points = np.round(object_2d_points).astype(int)
    object_2d_vertices, _ = cv2.projectPoints(object_3d_transform_vertices, np.eye(3), np.zeros(3), camera_matrix,
                                           distortion_coefficients)
    klt_2d_vertices = np.round(object_2d_vertices).astype(int)

    center_2d, _ = cv2.projectPoints(np.array([center_3d]), np.eye(3), np.zeros(3), camera_matrix,
                                     distortion_coefficients)
    center_2d = center_2d[0, 0]

    # create a mask by projecting the points on the image
    mask = np.zeros_like(img_temp_cam)
    for point in object_2d_points:
        mask = cv2.circle(mask, tuple(point[0].astype(int)), 2, (255, 255, 255), -1)
    # fill the mask with the projected points
    img_temp_cam = cv2.addWeighted(img_temp_cam, 1, mask, 0.3, 0)

    for point in klt_2d_vertices:
        img_temp_cam = cv2.circle(img_temp_cam, tuple(point[0].astype(int)), 3, (0, 0, 255), -1)
    img_temp_cam = cv2.circle(img_temp_cam, tuple(center_2d.astype(int)), 5, (255, 0, 0), -1)
    # cv2 show
    #cv2.imshow('img', cv2.resize(img_temp_cam, (0, 0), fx=0.5, fy=0.5))
    return img_temp_cam

def save_pose(H_cam_optical_2_point, center_3d, output_dir):
    rotation = H_cam_optical_2_point[:3, :3]
    rotmat = R.from_matrix(rotation)
    euler_angles = rotmat.as_euler('xyz', degrees=True)
    pose = np.concatenate((center_3d, euler_angles))

    file = os.path.join(output_dir, "pose.json")
    with open(file, 'a') as json_file:
        json.dump(pose.tolist(), json_file)
        json_file.write('\n')