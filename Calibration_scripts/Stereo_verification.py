import cv2
import numpy as np

# Given intrinsic parameters for cam0 and cam1 (replace with your actual values)
# For demonstration purposes, considering simple intrinsic parameters
intrinsics_cam0 = np.array([1388.49, 1376.86, 622.90, 236.41])  # fx, fy, cx, cy
intrinsics_cam1 = np.array([1253.41, 1294.94, 692.33, 210.14])  # fx, fy, cx, cy
K_cam0 = np.array([[1388.49, 0, 622.90], [0, 1376.86, 236.41], [0, 0, 1]])  # Intrinsic matrix
K_cam1 = np.array([[1253.4128932861238, 0, 692.3266533158668], [0, 1294.9362137466994, 210.1373077383432], [0, 0, 1]])  # Intrinsic matrix

dist_coeff_cam0 = np.array([-0.4029653799093595, 0.03528244676671536, 0.0003313602172192834, -0.03336916952285503])
dist_coeff_cam1 = np.array([-0.558541977698634, 0.09516835512412594, -0.0002777085810752234, -0.0867806077135203])
# undistorted_img = cv2.undistort(distorted_img, camera_matrix, distCoeffs=dist_coeffs)
                           # Transformation matrix T_cn_cnm1 for cam1 to cam0 (replace with your actual matrix)
T_cn_cnm1 = np.array([
    [0.9982913527536729, 0.0013741388258868403, -0.05841649390132803, 0.05808437086113932],
    [-0.0014347994895756596, 0.9999984741822838, -0.0009964855863416671, 0.0014547929071310666],
    [0.05841503545887324, 0.0010785988996209876, 0.9982918011567294, 0.007233432340630605],
    [0.0, 0.0, 0.0, 1.0]
])

# Generate some sample points in the coordinate system of cam1 (replace with your actual points)
points_cam0 = np.array([
    [1, 3, 1,1],  # Point 1 in homogeneous coordinates (x, y, z)
    [120, 230, 1,1],  # Point 2 in homogeneous coordinates (x, y, z)
    [240, 160, 1,1]   # Point 3 in homogeneous coordinates (x, y, z)
])

# Transform points from cam1 to cam0 using T_cn_cnm1
points_cam1 = np.dot(T_cn_cnm1, points_cam0.T).T

def rectify_img(intrinsics, dist_coeff, img):
    
    # Load the image
    #img = cv2.imread('/home/eventcamera/Eventcamera/calibration_data/stereo_event_calibration/cam1_orig/1702916577801071000.png')

    # Get the size of the image
    h = 480
    w = 640

    # Generate rectification map
    new_K, roi = cv2.getOptimalNewCameraMatrix(intrinsics, dist_coeff, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(intrinsics, dist_coeff, None, new_K, (w, h), 5)

    # Rectify the image
    rectified_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    return rectified_img

# Project points into image plane of cam0 and cam1 (visualize in the original images)
def project_points_into_image(points, intrinsics):
    fx, fy, cx, cy = intrinsics
    points_2d = points[:, :2] / points[:, 2:]  # Convert to 2D coordinates (x/z, y/z)
    points_2d[:, 0] = fx * points_2d[:, 0] + cx  # Scale and shift for image plane
    points_2d[:, 1] = fy * points_2d[:, 1] + cy
    return points_2d.astype(int)


# Project points into image planes of cam0 and cam1
#points_image_cam0 = project_points_into_image(points_cam0, intrinsics_cam0)
#points_image_cam1 = project_points_into_image(points_cam1, intrinsics_cam1)
points_image_cam0 = points_cam0[:, :2] / points_cam0[:, 2:]
points_image_cam0 = points_image_cam0.astype(int)
points_image_cam1 = points_cam1[:, :2] / points_cam1[:, 2:]
points_image_cam1 = points_image_cam1.astype(int)
# Visualize the points in the original images of cam0 and cam1 (for demonstration purposes)
img_cam1 = cv2.imread('/home/eventcamera/Eventcamera/calibration_data/stereo_event_calibration/cam1_orig/1702916568601071000.png')  # Replace with your actual image path for cam0
img_cam0 = cv2.imread('/home/eventcamera/Eventcamera/calibration_data/stereo_event_calibration/reconstructed_event_images/cam0/1702916568621072000.png')  # Replace with your actual image path for cam1
img_cam0 = rectify_img(K_cam0, dist_coeff_cam0, img_cam0)
img_cam1 = rectify_img(K_cam1, dist_coeff_cam1, img_cam1)

# Visualize points in the original images
for point in points_image_cam0:
    cv2.circle(img_cam1, tuple(point), 5, (0, 255, 0), -1)  # Green circles for cam0 points

for point in points_image_cam1:
    cv2.circle(img_cam1, tuple(point), 5, (255, 0, 0), -1)  # Blue circles for cam1 points

# Display the images with points
cv2.imshow('Cam0 Image with Points', img_cam0)
cv2.imshow('Cam1 Image with Points', img_cam1)
cv2.waitKey(0)
cv2.destroyAllWindows()
