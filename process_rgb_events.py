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
import numpy as np

# This scripts extracts the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw,
# /dvxplorer_right/events from the bag file.
# To extract RGB images, execute extract_rgb_img_from_bag.py Read the bag file
bag = rosbag.Bag('/home/eventcamera/data/test2.bag')
# Extract the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw, /dvxplorer_right/events
events_topic_left = '/dvxplorer_left/events'
events_topic_right = '/dvxplorer_right/events'
vicon_topic_cam_sys = '/vicon/event_cam_sys/event_cam_sys'
rgb_topic = '/rgb/image_raw'
events_left = []
events_right =[]
vicon = []
rgb = []
size = 400000


# Iterate over the bag file and extract the messages
#for topic, msg, t in bag.read_messages(topics=[events_topic_left, events_topic_right, vicon_topic_cam_sys, rgb_topic]):

    # events_left = bag.read_messages(events_topic_left)
count = 0
for top, msg, tim in bag.read_messages(events_topic_left):
    t = msg.header.stamp
    print(t)
    count = 0
    x = []
    y = []
    polarity = []
    for event_traverser in range(len(msg.events)):
        x.append((msg.events)[event_traverser].x)
        y.append((msg.events)[event_traverser].y)
        polarity.append((msg.events)[event_traverser].polarity)


        #image[(msg.events)[event_traverser].y, (msg.events)[event_traverser].x] = int(
        #    (msg.events)[event_traverser].polarity) * 254
        count += 1
    # save x,y polarity and timestamp in a .npy file
    
    np.save('/home/eventcamera/data/event.npy', [np.array(t), np.array(x), np.array(y), np.array(polarity)])
    print('saved')
events_right = bag.read_messages(events_topic_right)
vicon = bag.read_messages(vicon_topic_cam_sys)
image_topic = bag.read_messages(rgb_topic)

rgb.append(msg)
# Close the bag file
bag.close()



