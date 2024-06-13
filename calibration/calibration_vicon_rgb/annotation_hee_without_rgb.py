# Transform the coordinates in RGB camera frame to event camera frame
# RGB camera is cam0, Event camera 1 is cam1 and event camera 2 is cam2
import numpy as np
import cv2
import os
import json
from scipy.spatial.transform import Rotation as R
import trimesh
import open3d as o3d

# read .npy file


data_path = '/home/eventcamera/data/dataset/ciatronic_200_crane/vicon_data/'
json_path_camera_sys = os.path.join(data_path, 'event_cam_sys.json')
json_path_object = os.path.join(data_path, 'object1.json')
json_path_event_cam_left = '/home/eventcamera/data/dataset/ciatronic_200_crane/event_cam_left_220/e2calib/'
json_path_event_cam_right = '/home/eventcamera/data/dataset/ciatronic_200_crane/event_cam_right_220/e2calib/'
# with open(json_path_camera, 'r') as f:
#    data_camera = json.load(f)

rgb_image_path = '/home/eventcamera/data/dataset/ciatronic_200_crane/rgb/'
rgb_timestamp = os.listdir(rgb_image_path)
rgb_timestamp.sort()

event_cam_left_timestamp = os.listdir(json_path_event_cam_left)
event_cam_left_timestamp.sort()

event_cam_right_timestamp = os.listdir(json_path_event_cam_right)
event_cam_right_timestamp.sort()
H_cam_optical_2_base = np.eye(4)

with open(json_path_object, 'r') as file:
    object_array = json.load(file)
# extract only timestamp in a numpy array from dictionary loaded_array
timestamp_object = []

for k, v in object_array.items():
    timestamp_object.append(v['timestamp'])
timestamp_object = np.array(timestamp_object)


def find_closest_elements(A, B):
    result = {}

    # Ensure B is a set of integers for efficient removal and lookup
    B_set = {int(b) for b in B}

    for a in A:
        # Convert a to integer
        a_int = int(a)
        # Find the closest element in B
        closest_b = min(B_set, key=lambda x: abs(x - a_int))
        result[a] = closest_b
        B_set.remove(closest_b)

    return result


def remove_extension_and_convert_to_int(arr):
    # Remove ".png" extension and convert to integers
    modified_arr = [int(file_name[:-4]) for file_name in arr if file_name.endswith('.png')]
    return modified_arr


def trim_list(event_cam_timestamps, object_timestamps):
    # Find the smallest length between the two lists
    min_length = min(len(event_cam_timestamps), len(object_timestamps))

    # Slice the lists to the smallest length
    trimmed_event_cam_timestamps = event_cam_timestamps[:min_length]
    trimmed_object_items = list(object_timestamps.items())[:min_length]

    # Convert the sliced list of tuples back to a dictionary
    trimmed_object_timestamps = dict(trimmed_object_items)

    return trimmed_event_cam_timestamps, trimmed_object_timestamps


# Compute vicons coordinates of object corresponding to rgb image. The timestamp of the rgb image is used to find the
# nearest timestamp in vicon data of object
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
result_dict2 = find_closest_elements(rgb_timestamp, event_cam_left_timestamp)
timestamp_closest_ec_left = list(result_dict2.values())
result_dict3 = find_closest_elements(rgb_timestamp, event_cam_right_timestamp)
timestamp_closest_ec_right = list(result_dict3.values())

event_cam_left_timestamp, timestamp_object = trim_list(event_cam_left_timestamp, object_array)
#event_cam_left_timestamp, timestamp_object = trim_list(event_cam_left_timestamp, object_array)

# Associate timestamps in both event cameras to vicon object timestamps.
result_dict4 = find_closest_elements(event_cam_left_timestamp, timestamp_object)
timestamp_closest_ec_left_vicon = list(result_dict4.values())

#result_dict5 = find_closest_elements(event_cam_right_timestamp, timestamp_object)
#timestamp_closest_ec_right_vicon = list(result_dict5.values())

translations_with_timestamps = {
    timestamp: np.array(object_array[str(timestamp)]["translation"])
    for timestamp in timestamp_closest_ec_left_vicon}
rotations_with_timestamps = {
    timestamp: np.array(object_array[str(timestamp)]["rotation"])
    for timestamp in timestamp_closest_ec_left_vicon
}

#vicon_coord.append(loaded_array[str(value)])

# Transformation matrix obtained from eye in hand calibration
H_cam_vicon_2_cam_optical = np.array([[0.00563068, 0.03006136, 0.9995322, -0.05282819],
                                      [-0.99982796, -0.01749663, 0.00615856, 0.03674293],
                                      [0.01767357, -0.99939491, 0.02995767, 0.00407536],
                                      [0., 0., 0., 1.]])

# RGB camera
#params = [1.81601107e+03, 1.81264445e+03, 1.00383169e+03, 7.16010695e+02]
params = [2001.0250442780605, 2001.2767496004499, 970.1619103491635, 684.6369964551955]
# params = [2592.7798180209766, 2597.1074116646814, 1121.2441077660412, 690.1066893999352]
camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
#distortion_coefficients = np.array([-1.76581808e-01, 1.06210912e-01, -1.55074994e-04,
#                                    5.03366350e-04, -4.07696624e-02])
distortion_coefficients = np.array(
    [-0.16662668463462832, 0.09713587034707222, 0.00044649384097793574, 0.0006466275306382167])

camera_mtx_cam1 = np.array(
    [[726.8015187965628, 0, 275.7447750580928], [0, 725.2525019712525, 212.00552936834734], [0, 0, 1]])
# [[718.9289498879248, 0, 287.4206641081329], [0, 718.8476596505732, 232.6402787336837], [0, 0, 1]])
distortion_coeffs_cam1 = np.array(
    [-0.40846674226069263, 0.2067801786931747, 0.007207741354236219, 0.006261397706848751])
#[-0.3094967913882128, -0.10722657430965295, 0.008512403913427787, 0.000592616793055609])

camera_mtx_cam2 = np.array(
    [[756.1204892251881, 0, 283.43129078329116], [0, 754.9663110831402, 227.0824683283377], [0, 0, 1]])
#[[745.1353300950308, 0,  291.1070763508334], [0, 747.3744176138202, 245.89026445203564], [0, 0, 1]])
distortion_coeffs_cam2 = np.array(
    [-0.4107919894947944, 0.20240048255503923, 0.006230890760180581, 0.00970389087313616])
#[-0.2496983957244161, -0.2978060510925673, 0.009342174824708725, 0.00328860522240014])

#=============================================================================
# Transformation matrix from camera 1 to camera 0 and camera 2 to camera 1. cam0 is rgb camera, cam1 is event camera 1
# =============================================================================
quat_cam2_cam1 = [0.00026235, -0.00376717, -0.00030698, 0.99999282]
quat_cam1_cam0 = [-0.00916228, -0.04119687, 0.0005243, 0.9991089]
R_cam1_cam0 = R.from_quat(quat_cam1_cam0).as_matrix()

# Transformation matrix from camera 1 to camera 0
'''
H_cam1_2_rgb = np.array([[0.9971993087878418, 0.0005258497031806043, 0.07478811426377688, 0.053269788085482585],
                       [0.0006902602101232752, 0.9998679823207766, -0.016233960410603578, 0.009952490862473908],
                       [-0.07478677753376195, 0.016240117359809583, 0.99706729787625, -0.04082399413941046],
                       [0.0, 0.0, 0.0, 1.0]])

# Transformation matrix from camera 2 to camera 1
H_cam2_cam1 = np.array([[0.9999312261471128, -0.00039260427945142073, 0.011721298468554263, -0.10223038360927061],
                        [0.00039304051036216355, 0.9999999221500911, -3.491341559211893e-05, 0.00027606541625718685],
                        [-0.011721283848896005, 3.9517959594199465e-05, 0.9999313026119555, 0.005272335654555953],
                        [0.0, 0.0, 0.0, 1.0]])
'''
H_cam1_2_rgb = np.array([[0.9988487792874476, 0.0047032240817731635, 0.04773882905149164, 0.05547200068236508],
                         [-0.00480866672973969, 0.9999862455691012, 0.0020941339237770754, 0.004239383045238393],
                         [-0.04772832324996561, -0.0023212832324058085, 0.9988576569281016, 0.007983029405103982],
                         [0.0, 0.0, 0.0, 1.0]])

H_cam2_cam1 = np.array([[0.9999747401826854, 0.0016840625425892184, -0.006905282755123367, -0.1029018195612104],
                        [-0.0017160652568493136, 0.999987803370837, -0.004631223338073897, 0.0005686198826895407],
                        [0.006897399264200306, 0.004642956270043212, 0.9999654338228244, 0.00848555474834492],
                        [0.0, 0.0, 0.0, 1.0]])

# =============================================================================

transformations = {}
# Read the vicon coordinates of the even camera system. Traverse through the coordinates
with open(json_path_camera_sys, 'r') as f:
    data = json.load(f)
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
    H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])
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

    '''H_v_2_point = np.array([[1, 0, 0, t_x],
                            [0, 1, 0, t_y],
                            [0, 0, 1, t_z],
                            [0, 0, 0, 1]])'''

    H_v_2_point = np.eye(4)
    H_v_2_point[:3, :3] = rotation
    H_v_2_point[:3, 3] = [t_x, t_y, t_z]

    H_cam_optical_2_point = np.matmul(H_cam_optical_2_base, H_v_2_point)
    t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
    r_cam_optical_2_point = H_cam_optical_2_point[:3, :3]
    #print(t_cam_optical_2_point)
    # points_2d = cv2.projectPoints(t_cam_optical_2_point, np.eye(3), np.zeros(3), camera_matrix, distortion_coefficients)
    # points_2d = np.round(points_2d[0]).astype(int)
    # print(points_2d)
    # Display the 2d points on the image
    # img_test = cv2.circle(img_test, tuple(points_2d[0][0]), 10, (255, 0, 0), -1)

    # cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))  # resize image to 0.5 for display
    # cv2.waitKey(0)
    H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)
    # invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
    H_cam_optical_2_base = np.eye(4)
    H_cam_optical_2_base[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
    H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])
    # Compute translation t_cam_optical_2_base
    t_cam_optical_2_base = H_cam_optical_2_base[:3, 3]
    # t_cam_optical_2_base = np.transpose(H_base_2_cam_optical[:3, :3])
    H_rgb_2_point = H_cam_optical_2_point
    # project point (x,y,z) in cam0 coordinate to cam1 coordinate
    point_cam0 = np.array([
        [1, 0, 0, t_cam_optical_2_base[0]],
        [0, 1, 0, t_cam_optical_2_base[1]],
        [0, 0, 1, t_cam_optical_2_base[2]],
        [0, 0, 0, 1]])

    H_cam1_2_point = np.matmul(H_cam1_2_rgb, H_rgb_2_point)

    t_cam1_2_point = H_cam1_2_point[:3, 3]
    #print(t_cam1_2_point)
    # project point (863,819) in cam1 coordinate to cam2 coordinate
    H_cam2_2_point = np.matmul(H_cam2_cam1, H_cam1_2_point)
    t_cam2_2_point = H_cam2_2_point[:3, 3]
    #print(t_cam2_2_point)
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

with open('/home/eventcamera/data/transformations/transformations.json', 'w') as json_file:
    json.dump(transformations, json_file, indent=2)
print('saved transformations data')
'''
projections_cam = {}

# Transforms all point in vicon coordinates to camera optical or rgb frame
#for k,v in translations_with_timestamps.items():
for k, v in object_array.items():
    H_v_2_point = np.array([
        #[1, 0, 0, v[0]],
        [1, 0, 0, v['translation'][0]],
        [0, 1, 0, v['translation'][1]],
        [0, 0, 1, v['translation'][2]],
        [0, 0, 0, 1]])
    H_cam_optical_2_base = transformations[str(k)]['H_cam_optical_2_base']
    H_base_2_cam_vicon = transformations[str(k)]['H_base_2_cam_vicon']
    H_cam_optical_2_point = np.matmul(H_cam_optical_2_base, H_v_2_point)
    t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
    #print(t_cam_optical_2_point)
    # points_2d = cv2.projectPoints(t_cam_optical_2_point, np.eye(3), np.zeros(3), camera_matrix, distortion_coefficients)
    #points_2d = np.round(points_2d[0]).astype(int)
    #print(points_2d)
    # Display the 2d points on the image
    #img_test = cv2.circle(img_test, tuple(points_2d[0][0]), 10, (255, 0, 0), -1)

    #cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))  # resize image to 0.5 for display
    #cv2.waitKey(0)
    H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)
    # invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
    H_cam_optical_2_base = np.eye(4)
    H_cam_optical_2_base[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
    H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])
    # Compute translation t_cam_optical_2_base
    t_cam_optical_2_base = H_cam_optical_2_base[:3, 3]
    #t_cam_optical_2_base = np.transpose(H_base_2_cam_optical[:3, :3])
    H_rgb_2_point = H_cam_optical_2_point
    # project point (x,y,z) in cam0 coordinate to cam1 coordinate
    point_cam0 = np.array([
        [1, 0, 0, t_cam_optical_2_base[0]],
        [0, 1, 0, t_cam_optical_2_base[1]],
        [0, 0, 1, t_cam_optical_2_base[2]],
        [0, 0, 0, 1]])

    H_cam1_2_point = np.matmul(H_cam1_2_rgb, H_rgb_2_point)
    t_cam1_2_point = H_cam1_2_point[:3, 3]
    #print(point_cam1)
    # project point (863,819) in cam1 coordinate to cam2 coordinate
    H_cam2_2_point = np.matmul(H_cam2_cam1, H_cam1_2_point)
    t_cam2_2_point = H_cam2_2_point[:3, 3]
    # point_cam2 = np.matmul(H_cam2_cam1, point_cam1)
    #print(point_cam2)
    projections_cam[str(v['timestamp'])] = {'t_cam_optical_2_point': t_cam_optical_2_point.tolist(),
                                            't_cam_optical_2_base': t_cam_optical_2_base.tolist(),
                                            'point_event_cam_left': t_cam1_2_point.tolist(),
                                            'point_event_cam_right': t_cam2_2_point.tolist(),
                                            'timestamp': str(v['timestamp'])}

with open('/home/eventcamera/data/transformations/projections_cam.json', 'w') as json_file:
    json.dump(projections_cam, json_file, indent=2)
print('saved projection data')
'''
count = 0
with open('/home/eventcamera/data/transformations/transformations.json', 'r') as file:
    projected_point_rgb_ec1_ec2 = json.load(file)
kr = {}
vr = {}

for (k, v), (kr, vr) in zip(rotations_with_timestamps.items(), translations_with_timestamps.items()):
    print(kr)
    rgb_t = rgb_timestamp[count]
    # ec_left = timestamp_closest_ec_left[count]
    ec_left = timestamp_closest_ec_left_vicon[count]
    # ec_right = timestamp_closest_ec_right[count]
    rgb_img_path = "/home/eventcamera/data/dataset/ciatronic_200_crane/rgb/" + str(rgb_t) + ".png"
    event_cam_left = "/home/eventcamera/data/dataset/ciatronic_200_crane/event_cam_left_1000/e2calib/" + str(ec_left) + ".png"
    event_cam_right = "/home/eventcamera/data/dataset/ciatronic_200_crane/event_cam_right_1000/e2calib/" + str(ec_left) + ".png"
    H_v_2_point = np.array([
        [1, 0, 0, v[0]],
        [0, 1, 0, v[1]],
        [0, 0, 1, v[2]],
        [0, 0, 0, 1]])

    t_cam_optical_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['t_cam_optical_2_point'])
    H_cam_optical_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['H_cam_optical_2_point'])
    rotation = H_cam_optical_2_point[:3, :3]

    print(t_cam_optical_2_point)
    points_2d = cv2.projectPoints(t_cam_optical_2_point, np.eye(3), np.zeros(3), camera_matrix, distortion_coefficients)
    points_2d = np.round(points_2d[0]).astype(int)
    img_test = cv2.imread(rgb_img_path)
    img_test = cv2.circle(img_test, tuple(points_2d[0][0]), 10, (255, 0, 0), -1)

    obj_geometry = trimesh.load_mesh('/home/eventcamera/data/KLT/modified_obj_000010.ply')
    translation_vector = np.array([-100, -100, -100])

    if not isinstance(obj_geometry, trimesh.Trimesh):
        print("The object is not a Trimesh object. It is a", type(obj_geometry))

    obj_geometry.vertices += translation_vector
    trimesh_object = obj_geometry.convex_hull

    points_3d = np.array(trimesh_object.sample(3000)) / 1000
    vertices = np.array(trimesh_object.vertices) / 1000

    #points_3d = np.dot(points_3d, R.from_euler('z', 90, degrees=True).as_matrix())
    #vertices = np.dot(vertices, R.from_euler('z', 90, degrees=True).as_matrix())

    klt_3d_transform_points = np.matmul(H_cam_optical_2_point, np.vstack((points_3d.T, np.ones(points_3d.shape[0]))))[:3, :].T
    klt_3d_transform_vertices = np.matmul(H_cam_optical_2_point, np.vstack((vertices.T, np.ones(vertices.shape[0]))))[:3, :].T

    klt_2d_points, _ = cv2.projectPoints(klt_3d_transform_points, np.eye(3), np.zeros(3), camera_matrix,
                                         distortion_coefficients)
    klt_2d_vertices, _ = cv2.projectPoints(klt_3d_transform_vertices, np.eye(3), np.zeros(3), camera_matrix,
                                         distortion_coefficients)

    for point in klt_2d_points:
        img_test = cv2.circle(img_test, tuple(point[0].astype(int)), 1, (150, 150, 150), -1)
    for point in klt_2d_vertices:
        img_test = cv2.circle(img_test, tuple(point[0].astype(int)), 1, (0, 0, 255), -1)
   # cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))
   # cv2.waitKey(0)

    #H_cam_optical_2_base = np.eye(4)
    #H_cam_optical_2_base[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
    #H_cam_optical_2_base[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])
    # Compute translation t_cam_optical_2_base
    #t_cam_optical_2_base = H_cam_optical_2_base[:3, 3]
    #H_rgb_2_point = H_cam_optical_2_point
    # project point (x,y,z) in cam0 coordinate to cam1 coordinate
    #point_cam0 = np.array([
    #    [1, 0, 0, t_cam_optical_2_base[0]],
    #    [0, 1, 0, t_cam_optical_2_base[1]],
    #    [0, 0, 1, t_cam_optical_2_base[2]],
    #    [0, 0, 0, 1]])

    #H_cam1_2_point = np.matmul(H_cam1_2_rgb, H_rgb_2_point)
    #t_cam1_2_point = H_cam1_2_point[:3, 3]
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
        img_test1 = cv2.circle(img_test1, tuple(point[0].astype(int)), 1, (0, 0, 255), -1)

   # cv2.imshow('img', img_test1)  # resize image to 0.5 for display
   # cv2.waitKey(0)

    # print(point_cam1)
    # project point (863,819) in cam1 coordinate to cam2 coordinate
    #H_cam2_2_point = np.matmul(H_cam2_cam1, H_cam1_2_point)
    #t_cam2_2_point = H_cam2_2_point[:3, 3]
    t_cam2_2_point = np.array(projected_point_rgb_ec1_ec2[str(k)]['point_event_cam_right'])
    H_cam2_2_point = np.eye(4)
    H_cam2_2_point[:3, :3] = rotation
    H_cam2_2_point[:3, 3] = t_cam2_2_point
    # point_cam2 = np.matmul(H_cam2_cam1, point_cam1)
    # print(point_cam2)

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
        img_test2 = cv2.circle(img_test2, tuple(point[0].astype(int)), 1, (0, 0, 255), -1)

   # cv2.imshow('img', img_test2)  # resize image to 0.5 for display

    img_test = cv2.resize(img_test, (568, 426))
    img_test1 = cv2.resize(img_test1, (568, 426))
    img_test2 = cv2.resize(img_test2, (568, 426))

    concatenated_images = np.hstack((img_test, img_test1, img_test2))

    # Display the concatenated images
    cv2.imshow('Concatenated Images', concatenated_images)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    if k == 1712920784856162863:
        cv2.waitKey(0)
    cv2.waitKey(0)
    count += 1
    cv2.destroyAllWindows()
