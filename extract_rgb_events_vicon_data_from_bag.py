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
from datetime import datetime

# This scripts extracts the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw,
# /dvxplorer_right/events from the bag file.
# To extract RGB images, execute extract_rgb_img_from_bag.py Read the bag file

bag = rosbag.Bag('/home/eventcamera/data/dataset/aug16/kronen_2/kronen_2.bag')
# Extract the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw, /dvxplorer_right/events
events_topic_left = '/dvxplorer_left/events'
events_topic_right = '/dvxplorer_right/events'
vicon_topic_cam_sys = '/vicon/event_cam_sys/event_cam_sys'
vicon_object = '/vicon/kronen/kronen'
rgb_topic = '/rgb/image_raw'
events_left = []
events_right =[]
vicon = []
rgb = []
size = 400000
vicon_data = {}

# Iterate over the bag file and extract the messages
#for topic, msg, t in bag.read_messages(topics=[events_topic_left, events_topic_right, vicon_topic_cam_sys, rgb_topic]):

    # events_left = bag.read_messages(events_topic_left)
count = 0
'''
# Iterate over the bag file and extract the messages
for top, msg, tim in bag.read_messages(events_topic_left):
    t = msg.header.stamp
    count = 0
    x = []
    y = []
    polarity = []
    for event_traverser in range(len(msg.events)):
        x.append((msg.events)[event_traverser].x)
        y.append((msg.events)[event_traverser].y)
        polarity.append((msg.events)[event_traverser].polarity)
        count += 1
    # save x,y polarity and timestamp in a .npy file
    event_left_data = np.array([t, x, y, polarity], dtype=object)

    np.save('/home/eventcamera/data/dataset/ciatronic_200_crane/event_camera_left/' + str(t) + '.npy', event_left_data)

    #loaded_array = np.load('array_of_lists.npy', allow_pickle=True)
    #loaded_list1 = loaded_array[0]
    #loaded_list2 = loaded_array[1]
    #loaded_list3 = loaded_array[2]
print('saved event cam left')
for top, msg, tim in bag.read_messages(events_topic_right):
    t = msg.header.stamp
    count = 0
    x = []
    y = []
    polarity = []
    for event_traverser in range(len(msg.events)):
        x.append((msg.events)[event_traverser].x)
        y.append((msg.events)[event_traverser].y)
        polarity.append((msg.events)[event_traverser].polarity)
    count += 1
    # save x,y polarity and timestamp in a .npy file
    event_right_data = np.array([t, x, y, polarity], dtype=object)

    np.save('/home/eventcamera/data/dataset/ciatronic_200_crane/event_camera_right/' + str(t) + '.npy', event_right_data)
print('saved event cam right')
'''

count = 0
for top, msg, tim in bag.read_messages(vicon_topic_cam_sys):
    t = msg.header.stamp
    translation = [
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z]
    rotation = [
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w]
    # save t, translation and rotation to a json file
    vicon_data[count] = {'translation': translation, 'rotation': rotation, 'timestamp': str(t)}
    #vicon_data[str(t)] = {'translation': translation, 'rotation': rotation}
    count += 1

with open('/home/eventcamera/data/dataset/aug16/kronen_2/vicon_data/event_cam_sys.json', 'w') as json_file:
    json.dump(vicon_data, json_file, indent=2)
print('saved event cam data')

'''
for top, msg, tim in bag.read_messages(vicon_object):
    t = msg.header.stamp
    translation = [
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z]
    rotation = [
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w]
    # save t, translation and rotation to a json file
    vicon_data[str(t)] = {'translation': translation, 'rotation': rotation, 'timestamp': str(t)}
    #vicon_data[str(t)] = {'translation': translation, 'rotation': rotation}

with open('/home/eventcamera/data/dataset/aug16/kronen_2/vicon_data/object1.json', 'w') as json_file:
    json.dump(vicon_data, json_file, indent=2)
print('saved object data')
'''
#loaded_array = np.load('/home/eventcamera/data/vicon_data/object1.npy', allow_pickle=True)
#loaded_list1 = loaded_array[0]
#loaded_list2 = loaded_array[1]
#loaded_list3 = loaded_array[2]

'''
image_topic = bag.read_messages(rgb_topic)
for k, b in enumerate(image_topic):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(b.message, "bgr8")
    # cv_image.astype(np.uint8)

    # cv_image = cv_image[45:480,0:595]
    # cv_image = cv2.resize(cv_image, (640,480))
    cv2.imwrite('/home/eventcamera/data/dataset/aug16/kronen_2/rgb/' + str(b.timestamp) + '.png', cv_image)
    # print('saved: ',)

print('Done Extracting RGB images')
'''
# Close the bag file
bag.close()




