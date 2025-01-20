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
import rosbag
from dataClass import DataClass

objects_list = ['pallet', 'small_klt', 'big_klt', 'blue_klt', 'shogun_box', 'kronen_bier_crate', 'brinkhoff_bier_crate',
                'zivid_cardboard_box', 'dell_carboard_box', 'ciatronic_carboard_box', 'human', ' hupfwagon', 'mobile_robot']
obj = ['human_LH','human_RH', 'human_LL', 'human_RL', 'human_head', 'human_waist']
num = [1]
flag = 1
count = 0
folder_name = 'human_hupwagen_kronen_blue_klt_1'
vicon_data = {}
for k in num:
    for object in obj:

        print('Extracting data for object: ', object, ' with number: ', k)
        object_name = object

        # This scripts extracts the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw,
        # /dvxplorer_right/events from the bag file.
        # To extract RGB images, execute extract_rgb_img_from_bag.py Read the bag file
        bag = rosbag.Bag('/home/eventcamera/data/dataset/' + folder_name + '/' + folder_name + '.bag')
        path = '/home/eventcamera/data/dataset/' + folder_name + '/'

        # Extract the topics /dvxplorer_left/events, /vicon/event_cam_sys/event_cam_sys, /rgb/image_raw, /dvxplorer_right/events

        vicon_object = '/vicon/' + object + '/' + object
        vicon_human = '/vicon/markers'
        time_LH = []
        time_RH = []
        time_LL = []
        time_RL = []
        time_moroKopf = []
        size = 400000
        vicon_data = {}

            # events_left = bag.read_messages(events_topic_left)


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
        if object == 'human_LH':
            human_LH = DataClass(vicon_data)
        elif object == 'human_RH':
            human_RH = DataClass(vicon_data)
        elif object == 'human_LL':
            human_LL = DataClass(vicon_data)
        elif object == 'human_RL':
            human_RL = DataClass(vicon_data)
        elif object == 'human_head':
            human_head = DataClass(vicon_data)
        elif object == 'human_waist':
            human_waist = DataClass(vicon_data)

    def get_rot(obj_name,t):
        if obj_name == 'human_LH':
            rot = human_LH.get_rotation(t)
            return rot
        elif obj_name == 'human_RH':
            rot = human_RH.get_rotation(t)
            return rot
        elif obj_name == 'human_LL':
            rot = human_LL.get_rotation(t)
            return rot
        elif obj_name == 'human_RL':
            rot = human_RL.get_rotation(t)
            return rot
        elif obj_name == 'human_head':
            rot = human_head.get_rotation(t)
            return rot
        elif obj_name == 'human_waist':
            rot = human_waist.get_rotation(t)
            return rot


    max_x = -100000
    max_y = -100000
    max_z = -100000
    min_x = 100000
    min_y = 100000
    min_z = 100000
    save_flag = True
    vicon_object = '/vicon/markers'
    vicon_data = {}
    first_flag = 0
    save_index = []
    count = 0
    entry_flag = True
    bag = rosbag.Bag('/home/eventcamera/data/dataset/' + folder_name + '/' + folder_name + '.bag')

    for top, msg, tim in bag.read_messages(vicon_object):

        entry_flag = True
        count = count + 1
        t = msg.header.stamp
        if first_flag == 0:

            for j in range(int(len(msg.markers))):
                # if the marker_name contains human in the name string then save that value in a list
                if msg.markers[j].marker_name.find('human') != -1:
                    save_index.append(j)
            first_flag = 1

        for i in save_index:
            if  msg.markers[i].occluded == False:
                if entry_flag:
                    max_x = -100000
                    max_y = -100000
                    max_z = -100000
                    min_x = 100000
                    min_y = 100000
                    min_z = 100000
                    entry_flag = False
                save_flag = True
                # find the max and min x, y, z values of the markers among all save_index
                if msg.markers[i].translation.x > max_x:
                    max_x = msg.markers[i].translation.x
                    # truncate the last character of the marker_name to get the human part
                    rot = get_rot('human_head', t)
                if msg.markers[i].translation.y > max_y:
                    max_y = msg.markers[i].translation.y
                    rot = get_rot('human_head', t)
                if msg.markers[i].translation.z > max_z:
                    max_z = msg.markers[i].translation.z
                    rot = get_rot('human_head', t)
                if msg.markers[i].translation.x < min_x:
                    min_x = msg.markers[i].translation.x
                    rot = get_rot('human_head', t)
                if msg.markers[i].translation.y < min_y:
                    min_y = msg.markers[i].translation.y
                    rot = get_rot('human_head', t)
                if msg.markers[i].translation.z < min_z:
                    min_z = msg.markers[i].translation.z
                    rot = get_rot('human_head', t)
            else:
                print('msg' + str(count) + ' _ ' + str(i), ' is occluded')
                rot = get_rot('human_head', t)


        if save_flag:
            vicon_data[str(t)] = {'min_x': min_x, 'min_y': min_y, 'min_z': min_z, 'max_x': max_x, 'max_y': max_y,
                                  'max_z': max_z, 'rotation': rot, 'timestamp': str(t)}

            with open(path + '/vicon_data/human_bbox.json', 'w') as json_file:
                json.dump(vicon_data, json_file, indent=2)
    print('saved human bbox data')

    vicon_data = {}
    save_index = []
    first_flag = 0
    count = 0
    save_flag = True
    max_x = -100000
    max_y = -100000
    max_z = -100000
    min_x = 100000
    min_y = 100000
    min_z = 100000
    for top, msg, tim in bag.read_messages(vicon_object):

        entry_flag = True
        count = count + 1
        t = msg.header.stamp
        if first_flag == 0:

            for j in range(int(len(msg.markers))):
                # if the marker_name contains hupwagen in the name string then save that value in a list
                if msg.markers[j].marker_name.find('hupwagen') != -1:
                    save_index.append(j)
            first_flag = 1

        for i in save_index:
            if  msg.markers[i].occluded == False:
                if entry_flag:
                    max_x = -100000
                    max_y = -100000
                    max_z = -100000
                    min_x = 100000
                    min_y = 100000
                    min_z = 100000
                    entry_flag = False
                save_flag = True
                # find the max and min x, y, z values of the markers among all save_index
                if msg.markers[i].translation.x > max_x:
                    max_x = msg.markers[i].translation.x
                    # truncate the last character of the marker_name to get the human part

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

            else:
                print('msg' + str(count) + ' _ ' + str(i), ' is occluded')



        if save_flag:
            vicon_data[str(t)] = {'min_x': min_x, 'min_y': min_y, 'min_z': min_z, 'max_x': max_x, 'max_y': max_y,
                                  'max_z': max_z, 'timestamp': str(t)}

            with open(path + '/vicon_data/hupwagen.json', 'w') as json_file:
                json.dump(vicon_data, json_file, indent=2)
    print('saved hupwagen bbox data')
    bag.close()




