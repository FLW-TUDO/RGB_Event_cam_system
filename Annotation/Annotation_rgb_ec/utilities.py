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

def remove_delayed_timestamps(result_dict):
    keys_to_remove = []
    for key, value in result_dict.items():
        deviation = abs(int(key) - int(value))

        if deviation > 10000000:
            keys_to_remove.append(key)

    for key in keys_to_remove:
        del result_dict[key]
    return result_dict

def save_transformations(data, H_cam_vicon_2_cam_optical, object_array, H_cam1_2_rgb, H_cam2_cam1, path):
    transformations = {}
    for i, v in data.items():
        if i == str(len(data) - 1):
            continue
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
        H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]),
                                                 H_base_2_cam_optical[:3, 3])
        # Add H_cam_optical_2_base, timestamp to a dictionary. Append the H_cam_optical_2_base and timestamp on every iteration.
        # This will give a list of dictionaries with H_cam_optical_2_base and timestamp
        t_x = object_array[str(v['timestamp'])]['translation'][0]
        t_y = object_array[str(v['timestamp'])]['translation'][1]
        t_z = object_array[str(v['timestamp'])]['translation'][2]
        r_x = object_array[str(v['timestamp'])]['rotation'][0]
        r_y = object_array[str(v['timestamp'])]['rotation'][1]
        r_z = object_array[str(v['timestamp'])]['rotation'][2]
        r_w = object_array[str(v['timestamp'])]['rotation'][3]
        rotation = R.from_quat([r_x, r_y, r_z, r_w]).as_matrix()

        H_v_2_point = np.eye(4)
        H_v_2_point[:3, :3] = rotation
        H_v_2_point[:3, 3] = [t_x, t_y, t_z]

        H_cam_optical_2_point = np.matmul(H_cam_optical_2_base, H_v_2_point)
        t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
        H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)
        H_cam_optical_2_base = np.eye(4)
        H_cam_optical_2_base[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
        H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]),
                                                 H_base_2_cam_optical[:3, 3])
        t_cam_optical_2_base = H_cam_optical_2_base[:3, 3]
        H_rgb_2_point = H_cam_optical_2_point
        # project point (x,y,z) in cam0 coordinate to cam1 coordinate
        H_cam1_2_point = np.matmul(H_cam1_2_rgb, H_rgb_2_point)

        t_cam1_2_point = H_cam1_2_point[:3, 3]
        # print(t_cam1_2_point)
        # project point (863,819) in cam1 coordinate to cam2 coordinate
        H_cam2_2_point = np.matmul(H_cam2_cam1, H_cam1_2_point)
        t_cam2_2_point = H_cam2_2_point[:3, 3]
        # print(t_cam2_2_point)
        transformations[str(data[str(i)]['timestamp'])] = {'H_cam_optical_2_base': H_cam_optical_2_base.tolist(),
                                                           'H_cam_optical_2_point': H_cam_optical_2_point.tolist(),
                                                           'H_base_2_cam_vicon': H_base_2_cam_vicon.tolist(),
                                                           't_cam_optical_2_point': t_cam_optical_2_point.tolist(),
                                                           't_cam_optical_2_base': t_cam_optical_2_base.tolist(),
                                                           'point_event_cam_left': t_cam1_2_point.tolist(),
                                                           'point_event_cam_right': t_cam2_2_point.tolist(),
                                                           'rotation': rotation.tolist(),
                                                           'timestamp': str(v['timestamp'])
                                                           }
    with open(path + 'transformations.json', 'w') as json_file:
        json.dump(transformations, json_file, indent=2)
    print('saved transformations data')


def get_translated_points_vertice(object_id, vertices, points_3d):
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
        translation_vector = np.array([0, 0, 0.03])
        #translation_vector = np.array([0, 0.05, 0])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 8:
        translation_vector = np.array([0, 0, 0])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 9:
        rotation_matrix = R.from_euler('z', 90, degrees=True).as_matrix()
        vertices = np.dot(vertices, rotation_matrix)
        points_3d = np.dot(points_3d, rotation_matrix)
        translation_vector = np.array([0, 0.05, 0])
        vertices -= translation_vector
        points_3d -= translation_vector

    if object_id == 10:
        translation_vector = np.array([0, 0, 0])
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


def project_points_to_image_plane(object_3d_transform_points, object_3d_transform_vertices, center_3d, camera_matrix,distortion_coefficients, img_test):
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
    mask = np.zeros_like(img_test)
    for point in object_2d_points:
        mask = cv2.circle(mask, tuple(point[0].astype(int)), 5, (255, 255, 255), -1)
    # fill the mask with the projected points
    img_test = cv2.addWeighted(img_test, 1, mask, 0.3, 0)

    for point in klt_2d_vertices:
        img_test = cv2.circle(img_test, tuple(point[0].astype(int)), 8, (0, 0, 255), -1)
    img_test = cv2.circle(img_test, tuple(center_2d.astype(int)), 8, (0, 0, 255), -1)
    return img_test