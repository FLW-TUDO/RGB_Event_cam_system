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

# To extract RGB images, execute extract_rgb_img_from_bag.py
# Read the bag file
bag = rosbag.Bag('/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/third_calib/rgb_vicon.bag')
# Extract the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw, /dvxplorer_right/events
events_topic_left = '/dvxplorer_left/events'
events_topic_right = '/dvxplorer_right/events'
vicon_topic = '/vicon/event_cam_sys/event_cam_sys'
rgb_topic = '/rgb/image_raw'
events = []
vicon = []
rgb = []
# Iterate over the bag file and extract the messages
for topic, msg, t in bag.read_messages(topics=[events_topic_left, events_topic_right, vicon_topic, rgb_topic]):
    if topic == events_topic_left or topic == events_topic_right:
        events.append(msg)
    elif topic == vicon_topic:
        vicon.append(msg)
    elif topic == rgb_topic:
        rgb.append(msg)
# Close the bag file
bag.close()



