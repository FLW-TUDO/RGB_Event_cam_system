import numpy as np
import cv2
import glob
import os

# ---------------------- Load Images from Folders ----------------------
def load_images_from_folder(folder_path):
    image_paths = sorted(glob.glob(os.path.join(folder_path, "*.png")))  # Adjust extension if needed
    images = [cv2.imread(img_path) for img_path in image_paths]
    return images

# ---------------------- Detect Checkerboard Corners ----------------------
def detect_checkerboard_corners(image, pattern_size=(8, 5)):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    if ret:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    return ret, corners

# ---------------------- Calibrate Event Camera (Fisheye Model) ----------------------
def calibrate_fisheye(images, pattern_size=(8, 5), square_size=0.125):
    obj_points = []  # 3D world points
    img_points = []  # 2D image points

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    for i,img in enumerate(images):
        ret, corners = detect_checkerboard_corners(img, pattern_size)

        if ret:
            cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            cv2.imshow('Checkerboard', img)
            key = cv2.waitKey(0) & 0xFF  # Wait for user input

            if key == ord('y'):  # Press 'y' to accept
                obj_points.append(objp)
                img_points.append(corners)
                print(f"Accepted image {i + 1}")
            elif key == ord('n'):  # Press 'n' to reject
                print(f"Rejected image {i + 1}")
    # Convert to numpy array before passing to OpenCV
    obj_points = np.array(obj_points)
    img_points = np.array(img_points).astype(np.float32)

    K = np.zeros((3, 3), dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)

    h, w = images[0].shape[:2]

    ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        obj_points, img_points, (w, h), K, D, None, None,
        flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

    return K, D, rvecs, tvecs




# ---------------------- Calibrate RGB Camera (Pinhole Model) ----------------------
def calibrate_pinhole(images, pattern_size=(8, 5), square_size=0.125):
    obj_points = []  # 3D world points
    img_points = []  # 2D image points

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size  # Convert to real-world size

    for img in images:
        ret, corners = detect_checkerboard_corners(img, pattern_size)
        if ret:
            obj_points.append(objp)
            img_points.append(corners)

    K = np.zeros((3, 3))
    D = np.zeros((5, 1))  # Pinhole model uses 5 distortion coefficients

    h, w = images[0].shape[:2]

    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), K, D, None, None)

    return K, D, rvecs, tvecs

# ---------------------- Stereo Calibration ----------------------
def stereo_calibrate(K_event, D_event, K_rgb, D_rgb, event_images, rgb_images, pattern_size=(9, 6), square_size=0.064):
    obj_points = []  # List of object points
    img_points_event = []  # List of image points from event camera
    img_points_rgb = []  # List of image points from RGB camera

    # Create 3D points for the checkerboard
    objp = np.zeros((pattern_size[0] * pattern_size[1],1, 3), np.float32)  # Ensure CV_32FC3 type
    objp[:,0, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size  # Scale by square size
    valid_pairs = 0
    h_event, w_event = event_images[0].shape[:2]
    resized_rgb_images = [cv2.resize(rgb_img, (w_event, h_event)) for rgb_img in rgb_images]

    # Debugging code to visualize detected corners
    for event_img, rgb_img in zip(event_images, resized_rgb_images):
        ret_event, corners_event = detect_checkerboard_corners(event_img, pattern_size)
        ret_rgb, corners_rgb = detect_checkerboard_corners(rgb_img, pattern_size)
        if ret_event and ret_rgb:
            valid_pairs += 1
            # Visualize detected corners
            cv2.drawChessboardCorners(event_img, pattern_size, corners_event, ret_event)
            cv2.drawChessboardCorners(rgb_img, pattern_size, corners_rgb, ret_rgb)
            cv2.imshow('Event Image', event_img)
            cv2.imshow('RGB Image', rgb_img)
            cv2.waitKey(0)
            obj_points.append(objp)
            img_points_event.append(corners_event)
            img_points_rgb.append(corners_rgb)
    cv2.destroyAllWindows()

    # Ensure distortion coefficients have the correct shape (4,1)
    D_event = np.array(D_event, dtype=np.float64).reshape(4, 1)
    D_rgb = np.array(D_rgb[:4], dtype=np.float64).reshape(4, 1)  # Ensure 4 elements

    print("K_event:", np.isnan(K_event).any(), np.isinf(K_event).any())
    print("D_event:", np.isnan(D_event).any(), np.isinf(D_event).any())
    print("K_rgb:", np.isnan(K_rgb).any(), np.isinf(K_rgb).any())
    print("D_rgb:", np.isnan(D_rgb).any(), np.isinf(D_rgb).any())
    # Call stereo calibration
    ret, K_event, D_event, K_rgb, D_rgb, R, T = cv2.fisheye.stereoCalibrate(
        obj_points, img_points_event, img_points_rgb,
        K_event, D_event, K_rgb, D_rgb,
        event_images[0].shape[:2][::-1], None, None,
        flags=cv2.fisheye.CALIB_FIX_INTRINSIC

    )

    return ret, K_event, D_event, K_rgb, D_rgb, R, T

# ---------------------- RUN THE FULL PIPELINE ----------------------
# Define folder paths (Update these paths)
event1_folder = "/media/eventcamera/event_data/calibration/mar_13/cam0_trim"
event2_folder = "/media/eventcamera/event_data/calibration/mar_12/cam2_trim"
rgb_folder = "/media/eventcamera/event_data/calibration/mar_12/cam1_trim"
print("Loading images from folders completed")

# Load images from each camera's folder
event_images1 = load_images_from_folder(event1_folder)
event_images2 = load_images_from_folder(event2_folder)
rgb_images = load_images_from_folder(rgb_folder)

# Intrinsic Calibration
K1, D1, rvecs1, tvecs1 = calibrate_fisheye(event_images1)
print("Intrinsic calibration for event camera 1 completed")
K2, D2, rvecs2, tvecs2 = calibrate_fisheye(event_images2)
print("Intrinsic calibration for event camera 2 completed")
K_rgb, D_rgb, rvecs_rgb, tvecs_rgb = calibrate_fisheye(rgb_images)
print("Intrinsic calibration for RGB camera completed")

# Extrinsic Calibration
R1_rgb, T1_rgb = stereo_calibrate(K1, D1, K_rgb, D_rgb, event_images1, rgb_images)
print("Extrinsic calibration for event camera 1 to rgb images completed")
R2_rgb, T2_rgb = stereo_calibrate(K2, D2, K_rgb, D_rgb, event_images2, rgb_images)
print("Extrinsic calibration for event camera 2 to rgb images completed")

# Print Results
print("Event Camera 1 to RGB Camera:")
print("Rotation Matrix:\n", R1_rgb)
print("Translation Vector:\n", T1_rgb)

print("\nEvent Camera 2 to RGB Camera:")
print("Rotation Matrix:\n", R2_rgb)
print("Translation Vector:\n", T2_rgb)
