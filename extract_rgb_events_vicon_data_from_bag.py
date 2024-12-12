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

objects_list = ['pallet', 'small_klt', 'big_klt', 'blue_klt', 'shogun_box', 'kronen_bier_crate', 'brinkhoff_bier_crate',
                'zivid_cardboard_box', 'dell_carboard_box', 'ciatronic_carboard_box', 'human', ' hupfwagon', 'mobile_robot']
obj = ['aerobie']
num = [1]
flag = 1
rgb_topic = '/rgb/image_raw'

for k in num:
    for object in obj:
        print('Extracting data for object: ', object, ' with number: ', k)
        object_name = object + '_' + str(k)

        # This scripts extracts the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw,
        # /dvxplorer_right/events from the bag file.
        # To extract RGB images, execute extract_rgb_img_from_bag.py Read the bag file
        path = '/home/eventcamera/data/dataset/' + object_name + '/'
        if(len(obj) > 1):
            object_name = obj[0] + '_' + obj[1] + '_' + str(k)
            bag = rosbag.Bag('/home/eventcamera/data/dataset/' + object_name + '/' + object_name + '.bag')
        path = '/home/eventcamera/data/dataset/' + object_name + '/'
        bag = rosbag.Bag('/home/eventcamera/data/dataset/' + object_name + '/' + object_name + '.bag')
        # Extract the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw, /dvxplorer_right/events
        events_topic_left = '/dvxplorer_left/events'
        events_topic_right = '/dvxplorer_right/events'
        vicon_topic_cam_sys = '/vicon/event_cam_sys/event_cam_sys'
        vicon_object = '/vicon/' + object + '/' + object
        vicon_human = '/vicon/markers'

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
        if flag == 1:
            ''''
            # mkdir if path does not exist
            if not os.path.exists(path + 'event_cam_left_npy'):
                os.mkdir(path + 'event_cam_left_npy')
            # Iterate over the bag file and extract the messages
            for top, msg, tim in bag.read_messages(events_topic_left):
                t = msg.header.stamp
                count = len(msg.events)
                event_data = np.zeros(count, dtype=[('t', 'float64'), ('x', 'int32'), ('y', 'int32'), ('p', 'int8')])

                x = []
                y = []
                polarity = []
                for i, event in enumerate(msg.events):
                    event_data[i] = (str(t), event.x, event.y, event.polarity)
                # save x,y polarity and timestamp in a .npy file
                #event_left_data = np.array([t, x, y, polarity], dtype=float)

                np.save(path + 'event_cam_left_npy/' + str(t) + '.npy', event_data)
                #print(count)
                #loaded_array = np.load('array_of_lists.npy', allow_pickle=True)
                #loaded_list1 = loaded_array[0]
                #loaded_list2 = loaded_array[1]
                #loaded_list3 = loaded_array[2]
            print('saved event cam left npy files')
            if not os.path.exists(path + 'event_cam_right_npy'):
                os.mkdir(path + 'event_cam_right_npy')
            for top, msg, tim in bag.read_messages(events_topic_right):
                t = msg.header.stamp
                count = len(msg.events)
                event_data = np.zeros(count, dtype=[('t', 'float64'), ('x', 'int32'), ('y', 'int32'), ('p', 'int8')])

                x = []
                y = []
                polarity = []
                for i, event in enumerate(msg.events):
                    event_data[i] = (str(t), event.x, event.y, event.polarity)
                # save x,y polarity and timestamp in a .npy file
                # event_left_data = np.array([t, x, y, polarity], dtype=float)

                np.save(path + 'event_cam_right_npy/' + str(t) + '.npy', event_data)
            print('saved event cam right npy files')

            count = 0
            if not os.path.exists(path + '/vicon_data'):
                os.makedirs(path + '/vicon_data')
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

            with open(path + '/vicon_data/event_cam_sys.json', 'w') as json_file:
                json.dump(vicon_data, json_file, indent=2)
            print('saved event cam data')

            image_topic = bag.read_messages(rgb_topic)
            if not os.path.exists(path + '/rgb'):
                os.makedirs(path + '/rgb')
            for l, b in enumerate(image_topic):
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(b.message, "bgr8")
                # cv_image.astype(np.uint8)

                # cv_image = cv_image[45:480,0:595]
                # cv_image = cv2.resize(cv_image, (640,480))
                cv2.imwrite(path + '/rgb/' + str(b.timestamp) + '.png', cv_image)
                print('saved: images',)
            print('Done Extracting RGB images')

            flag = 0

        if not os.path.exists(path + '/vicon_data'):
            os.makedirs(path + '/vicon_data')
        vicon_data = {}
        print('Extracting data for object: ', object, ' with number: ', k)
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
            # vicon_data[str(t)] = {'translation': translation, 'rotation': rotation}

        with open(path + 'vicon_data/' + object + '_' + str(k) + '.json', 'w') as json_file:
            json.dump(vicon_data, json_file, indent=2)
            print('saved object data')
            # Close the bag file
'''
max_x = -100000
max_y = -100000
max_z = -100000
min_x = 100000
min_y = 100000
min_z = 100000
vicon_object = '/vicon/markers'
vicon_data = {}
first_flag = 0
save_index = []
bag = rosbag.Bag('/home/eventcamera/data/dataset/' + object_name + '/' + object_name + '.bag')
for top, msg, tim in bag.read_messages(vicon_object):
    t = msg.header.stamp
    if first_flag == 0:

        for j in range(int(len(msg.markers))):
            # if the marker_name contains human in the name string then save that value in a list
            if msg.markers[j].marker_name.find('aero') != -1:
                save_index.append(j)
        first_flag = 1

    for i in save_index:
        # find the max and min x, y, z values of the markers among all save_index
        if msg.markers[i].translation.x > max_x:
            max_x = msg.markers[i].translation.x
        if msg.markers[i].translation.y > max_y:
            max_y = msg.markers[i].translation.y
        if msg.markers[i].translation.z > max_z:
            max_z = msg.markers[i].translation.z
        if msg.markers[i].translation.x < min_x:
            min_x = msg.markers[i].translation.x
        if msg.markers[i].translation.y < min_y:
            min_y = msg.markers[i].translation.y
        if msg.markers[i].translation.z < min_z:
            min_z = msg.markers[i].translation.z

    # calculate the mean of all the markers
    vicon_data[str(t)] = {'min_x': min_x, 'min_y': min_y, 'min_z': min_z, 'max_x': max_x, 'max_y': max_y,
                          'max_z': max_z, 'timestamp': str(t)}

    with open('/home/eventcamera/data/dataset/pallet_ec_blue_klt_1/' + 'human_bbox.json', 'w') as json_file:
        json.dump(vicon_data, json_file, indent=2)
        print('saved human bbox data')
bag.close()




