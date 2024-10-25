#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  5 14:27:36 2024

@author: Shrutarv Awasthi
"""

import rospy
from cv_bridge import CvBridge
import json

from sphinx.builders.gettext import timestamp
from std_msgs.msg import Float32MultiArray
import cv2
import os
import time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

# Initialize ROS node
rospy.init_node('image_and_vicon_subscriber')

# Initialize CvBridge
bridge = CvBridge()

# Variables to store image and vicon coordinates
received_image = None
received_vicon = None

# Callback function for image
def image_callback(data):
    global received_image
    received_image = bridge.imgmsg_to_cv2(data, "bgr8")

# Callback function for vicon coordinates
def vicon_callback(data):
    global received_vicon
    received_vicon = data

# Subscribers for image and vicon coordinates
image_subscriber = rospy.Subscriber('/rgb/image_raw', Image, image_callback)
vicon_subscriber = rospy.Subscriber('/vicon/event_cam_sys/event_cam_sys', TransformStamped, vicon_callback)

# Folder to save images
save_folder = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/calib_23_oct/'
os.makedirs(save_folder, exist_ok=True)
image_dir = os.path.join(save_folder, 'images')
os.makedirs(image_dir, exist_ok=True)
json_file_path = os.path.join(save_folder, 'vicon_coordinates.json')

vicon_data = {}


# Loop to capture and save image and vicon data on 'Enter' key press
count = 0
while not rospy.is_shutdown():
    key = input("press Enter to capture sample: " + str(count))
    if key == 'q':
        break
    time.sleep(0.1)
    # Save image to folder
    image_filename = str(count) + '.png'
    #cv2.imwrite(os.path.join(image_dir, image_filename), received_image)
    # extract the timestamp from the vicon data and save as nano secs
    timestamp = received_vicon.header.stamp.secs + received_vicon.header.stamp.nsecs * 1e-9
    # delete the decimal point and join bothe the strings
    #timestamp = str(timestamp).replace('.','')
    # Save vicon coordinates to a JSON file
    translation = [
        received_vicon.transform.translation.x,
        received_vicon.transform.translation.y,
        received_vicon.transform.translation.z]
    rotation = [
        received_vicon.transform.rotation.x,
        received_vicon.transform.rotation.y,
        received_vicon.transform.rotation.z,
        received_vicon.transform.rotation.w,
        ]
    vicon_data[count] = {'timestamp': timestamp, 'translation': translation, 'rotation': rotation}

    count += 1


with open(json_file_path, 'w') as json_file:
    json.dump(vicon_data, json_file)

# Close OpenCV windows
cv2.destroyAllWindows()
