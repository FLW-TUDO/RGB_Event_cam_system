import numpy as np
import cv2
import json

root = "/media/eventcamera/event_data/dataset_31_march_zft/scene67/"
json_file = "/media/eventcamera/event_data/dataset_31_march_zft/scene67/output_masks_human_img/bounding_boxes.json"
with open(json_file, "r") as f:
    data = json.load(f)

with open(root + 'camera_params.json', 'r') as file:
    cam_params = json.load(file)
# Extract camera intrinsic matrix
K_left = np.array(cam_params['camera_matrix'])
fx = K_left[0][0]
fy = K_left[1][1]
cx = K_left[0][2]
cy = K_left[1][2]
dist_coefficients_left = np.array(cam_params['distortion_coefficients'])

K_rgb = np.array(cam_params['camera_mtx_cam1'])
K_right = np.array(cam_params['camera_mtx_cam2'])
fx_rgb = K_rgb[0][0]
fy_rgb = K_rgb[1][1]
cx_rgb = K_rgb[0][2]
cy_rgb = K_rgb[1][2]
dist_coefficients_rgb = np.array(cam_params['distortion_coeffs_cam1'])

H_rgb_to_left = np.array(cam_params['H_rgb_2_left'])
H_right_to_rgb = np.array(cam_params['H_right_2_rgb'])
H_rgb_to_right = np.linalg.inv(H_right_to_rgb)


def pixel_to_camera(x_pixel, y_pixel, z, fx, fy, cx, cy):
    X = (x_pixel - cx) * z / fx
    Y = (y_pixel - cy) * z / fy
    return X, Y, z


rvec = np.zeros(3)  # rotation vector
tvec = np.zeros(3)
# loop throught he list data
for entry in data:
    xmin = entry["xmin"]
    ymin = entry["ymin"]
    xmax = entry["xmax"]
    ymax = entry["ymax"]
    x_coord_left = []
    y_coord_left = []
    x_coord_right = []
    y_coord_right = []
    z = entry["zmax"]
    coordinate_bbox = [(xmin,ymin), (xmin,ymax), (xmax,ymin), (xmax,ymax)]
    # convert all the pixel coordinate_bbox to camera coordinates
    for coord in coordinate_bbox:
        x_pixel = coord[0]
        y_pixel = coord[1]
        X, Y, Z = pixel_to_camera(x_pixel, y_pixel, z, fx_rgb, fy_rgb, cx_rgb, cy_rgb)
        coords_left = H_rgb_to_left @ np.array([X, Y, Z, 1])
        coords_right = H_rgb_to_right @ np.array([X, Y, Z, 1])
        points_3d_left = np.array([[coords_left[0], coords_left[1], coords_left[2]]])
        points_2d_left, _ = cv2.projectPoints(points_3d_left, rvec, tvec, K_left, dist_coefficients_left)
        # round the coordinates X,Y and append to a list
        x_coord_left.append((np.round(points_2d_left[0][0][0]).astype(int)))
        y_coord_left.append((np.round(points_2d_left[0][0][1]).astype(int)))
        x_coord_right.append((np.round(points_2d_left[0][0][0]).astype(int)))
        y_coord_right.append((np.round(points_2d_left[0][0][1]).astype(int)))
    xmin_left = min(x_coord_left)
    ymin_left = min(y_coord_left)
    xmax_left = max(x_coord_left)
    ymax_left = max(y_coord_left)

    xmin_right = min(x_coord_right)
    ymin_right = min(y_coord_right)
    xmax_right = max(x_coord_right)
    ymax_right = max(y_coord_right)

    entry["xmin_left"] = int(xmin_left)
    entry["ymin_left"] = int(ymin_left)
    entry["xmax_left"] = int(xmax_left)
    entry["ymax_left"] = int(ymax_left)

    entry["xmin_right"] = int(xmin_right)
    entry["ymin_right"] = int(ymin_right)
    entry["xmax_right"] = int(xmax_right)
    entry["ymax_right"] = int(ymax_right)
# save the new json file with the new coordinates
with open(root + 'output_masks_human_img/bounding_boxes_2D.json', 'w') as f:
    json.dump(data, f, indent=2)

    print("done")