#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  5 14:27:36 2024

@author: Shrutarv Awasthi
"""

import rospy
from cv_bridge import CvBridge
import json
from std_msgs.msg import Float32MultiArray
import cv2
import os
import time
import rosbag
from dvs_msgs.msg import EventArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

# Initialize ROS node
rospy.init_node('image_and_vicon_subscriber')

# Initialize CvBridge
bridge = CvBridge()

# Variables to store image and vicon coordinates
received_image = None
received_vicon = None
event_topic = ['/dvxplorer_left/events, /dvxplorer_right/events']

# Callback function for image
def image_callback(data):
    global received_image
    global i
    received_image = bridge.imgmsg_to_cv2(data, "bgr8")
    i = i + 1
    cv_image = bridge.imgmsg_to_cv2(received_image, desired_encoding='passthrough')
    rgb_timestamp = int(received_image.header.stamp.nsecs )#/ 1000) + rgb_data.header.stamp.secs * 1000000
    #rgb_timestamp *= 1000
    cv2.imwrite(save_folder + '/' + str(rgb_timestamp) + '.png', cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

# Callback function for vicon coordinates
def vicon_callback(data):
    global received_vicon
    received_vicon = data

def event_callback(event_data):
    bag.write(event_topic, event_data)

# Subscribers for image and vicon coordinates

vicon_subscriber = rospy.Subscriber('/vicon/event_cam_sys/event_cam_sys', TransformStamped, vicon_callback)

# Folder to save images
save_folder = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration'
os.makedirs(save_folder, exist_ok=True)
image_dir = os.path.join(save_folder, 'images')
os.makedirs(image_dir, exist_ok=True)
json_file_path = os.path.join(save_folder, 'vicon_coordinates.json')

vicon_data = {}


# Loop to capture and save two ros topics /dvxplorer_left/events and /dvxplorer_right/events in a ros bag

while not rospy.is_shutdown():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/rgb/image_raw', Image, image_callback)
    # write a subscriber for two event topics
    event_sub = rospy.Subscriber(event_topic, EventArray, event_callback)

    bag = rosbag.Bag(save_folder + 'events.bag', 'w')

    rospy.spin()


with open(json_file_path, 'w') as json_file:
    json.dump(vicon_data, json_file)

# Close OpenCV windows
cv2.destroyAllWindows()
