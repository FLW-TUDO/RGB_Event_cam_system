import cv2
import numpy as np

# Load the calibration parameters
# Replace these with your actual calibration parameters
K = np.array([[1253.4128932861238, 0, 692.3266533158668], [0, 1294.9362137466994, 210.1373077383432], [0, 0, 1]])  # Intrinsic matrix

dist_coeff = np.array([-0.558541977698634, 0.09516835512412594, -0.0002777085810752234, -0.0867806077135203])  # Distortion coefficients

# Load the image
img = cv2.imread('/home/eventcamera/Eventcamera/calibration_data/stereo_event_calibration/cam1_orig/1702916577801071000.png')

# Get the size of the image
h, w = img.shape[:2]

# Generate rectification map
new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist_coeff, (w, h), 1, (w, h))
mapx, mapy = cv2.initUndistortRectifyMap(K, dist_coeff, None, new_K, (w, h), 5)

# Rectify the image
rectified_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

# Crop the image to remove black borders
x, y, rectified_w, rectified_h = roi
rectified_img = rectified_img[y:y + rectified_h, x:x + rectified_w]

# Display the original and rectified images
cv2.imshow('Original Image', img)
cv2.imshow('Rectified Image', rectified_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
