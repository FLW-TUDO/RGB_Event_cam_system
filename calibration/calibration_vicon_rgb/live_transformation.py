#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  5 14:27:36 2024

@author: Shrutarv Awasthi
"""
import numpy as np
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
import sys
from threading import Thread
from scipy.spatial.transform import Rotation as R

# Initialize ROS node
rospy.init_node('image_and_vicon_subscriber')

# Initialize CvBridge
bridge = CvBridge()

# Variables to store image and vicon coordinates
received_image = None
received_vicon = None
m_device = None
event_topic_left = '/dvxplorer_left/events'
event_topic_right = '/dvxplorer_right/events'
size = 400000
x_left = np.zeros(size)
y_left = np.zeros(size)
x_right = np.zeros(size)
y_right = np.zeros(size)
polarity_left = np.zeros(size)
polarity_right = np.zeros(size)
image_left = np.zeros((480, 640))
image_right = np.zeros((480, 640))

vicon_cam_rotation =[]
vicon_cam_translation = []
event_right = []
event_left = []
received_vicon_data = None
received_vicon_obj_data = None
json_file_event_left_path = '/home/eventcamera/data/event_left.json'
json_file_event_right_path = '/home/eventcamera/data/event_right.json'
json_file_vicon_path = '/home/eventcamera/data/vicon.json'
json_file_rgb_path = '/home/eventcamera/data/rgb.json'
flag_event_left = 0
flag_event_right = 0
flag_vicon = 0
flag_rgb = 0

# Callback function for image
def image_callback(data):
    print('image callback')
    global received_image, flag_rgb
    # global i
    received_image = bridge.imgmsg_to_cv2(data, "bgr8")
    if received_image is not None:
        flag_rgb = 1
    # i = i + 1
    #cv_image = bridge.imgmsg_to_cv2(received_image, "bgr8")
    # rgb_timestamp = int(received_image.header.stamp.nsecs)  # / 1000) + rgb_data.header.stamp.secs * 1000000
    # rgb_timestamp *= 1000
    # cv2.imwrite(save_folder + '/' + str(rgb_timestamp) + '.png', cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))


# Callback function for vicon coordinates
def vicon_callback(data):
    global vicon_cam_rotation, vicon_cam_translation, flag_vicon
    received_vicon_data = data
    if received_vicon_data is not None:
        flag_vicon = 1
    vicon_cam_translation = [
        received_vicon_data.transform.translation.x,
        received_vicon_data.transform.translation.y,
        received_vicon_data.transform.translation.z]
    vicon_cam_rotation = [
        received_vicon_data.transform.rotation.x,
        received_vicon_data.transform.rotation.y,
        received_vicon_data.transform.rotation.z,
        received_vicon_data.transform.rotation.w,
    ]
    #print(vicon_cam_rotation)

def event_callback_left(event_data):
    # read data from
    global event_left, flag_event_left
    flag_event_left = 1
    print('left callback')
    event_left = event_data.events
    event_callback_left.counter += 1
    #if event_callback_left.counter % 5 == 0:
    #    compute_transformation(event_left, event_right, received_vicon_data, received_image)


def event_callback_right(event_data_right):
    # read data from
    global event_right, flag_event_right
    flag_event_right = 1
    event_right = event_data_right.events
    # print('inside callback right')

def vicon_obj_callback(data):
    global received_vicon_obj_data
    received_vicon_obj_data = data

def callback_thread_left():
    # rospy.init_node('callback_node', anonymous=True)  # Initialize node in the thread
    rospy.Subscriber(event_topic_left, EventArray, event_callback_left)
    rospy.spin()


def callback_thread_right():
    # rospy.init_node('callback_node', anonymous=True)  # Initialize node in the thread
    rospy.Subscriber(event_topic_right, EventArray, event_callback_right)
    rospy.spin()


def callback_thread_image():
    # rospy.init_node('callback_node', anonymous=True)  # Initialize node in the thread
    rospy.Subscriber('/rgb/image_raw', Image, image_callback)
    rospy.spin()


def callback_thread_vicon():
    # rospy.init_node('callback_node', anonymous=True)  # Initialize node in the thread
    rospy.Subscriber('/vicon/event_cam_sys/event_cam_sys', TransformStamped, vicon_callback)
    rospy.spin()

    # bag.write(event_topic, event_data)


# Function to execute your scripts
def compute_transformation(event_l, event_r, vicon_cam_r, vicon_cam_t, received_img):
    # Add your script execution logic here
    print("Computing transformations after receiving 5 messages")
    for event_traverser in range(len(event_l)):
        x_left[event_traverser] = event_l[event_traverser].x
        y_left[event_traverser] = event_l[event_traverser].y
        polarity_left[event_traverser] = event_l[event_traverser].polarity
        image_left[event_l[event_traverser].y, event_l[event_traverser].x] = int(
            event_l.events[event_traverser].polarity) * 254
        # visualize image using opencv
        cv2.imshow('image', image_left)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()
    for event_traverser in range(len(event_r)):
        x_right[event_traverser] = event_r[event_traverser].x
        y_right[event_traverser] = event_r[event_traverser].y
        polarity_right[event_traverser] = event_r[event_traverser].polarity
        image_right[event_r[event_traverser].y, event_r[event_traverser].x] = int(
            event_r.events[event_traverser].polarity) * 254
        # visualize image using opencv
        cv2.imshow('image', image_right)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()


    H_cam_vicon_2_cam_optical = np.array([
        [0.1844511, 0.10184152, 0.97755107, 0.13975843],
        [-0.98279335, 0.02897629, 0.18242149, -0.54963646],
        [-0.00974772, -0.99437854, 0.10543387, -0.50285077],
        [0., 0., 0., 1.]])

    params = [1.93686226e+03, 1.93330361e+03, 9.85688080e+02, 7.71096964e+02]
    camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
    distortion_coefficients = np.array([-0.12905658, -0.01019267, 0.00562304, -0.00015354, 0.13542021])

    # base to camera_vicon transformation. The below values are taken from recorded json file
    # get rotation matrix from quaternion
    rotation = R.from_quat(vicon_cam_r).as_matrix()
    # make homogeneous transformation matrix
    H_base_2_cam_vicon = np.eye(4)
    H_base_2_cam_vicon[:3, :3] = rotation
    H_base_2_cam_vicon[:3, 3] = vicon_cam_t

    # make homogeneous transformation matrix from base to camera optical frame
    H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)

    # invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
    H_cam_optical_2_vicon = np.eye(4)
    H_cam_optical_2_vicon[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
    H_cam_optical_2_vicon[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])

    # TODO: get the translation and rotation vec from the moving object by subscribing to the vicon topic
    H_v_2_point = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])
    # transform the point to the camera optical frame
    H_cam_optical_2_point = np.matmul(H_cam_optical_2_vicon, H_v_2_point)

    # get the 3d point
    t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
    t_rgb_2_point = t_cam_optical_2_point
    H_rgb_2_point = H_cam_optical_2_point

    H_cam1_rgb = np.array([[0.9966050852350032, 0.0018025810956462127, 0.08231072096103041, 0.058348558249773974],
                           [-0.00029275262105358537, 0.9998315556310521, -0.018351421206337717, 0.0015723475769044038],
                           [-0.0823299361085253, 0.018265023016223507, 0.9964377404306713, 0.02935723981670002],
                           [0.0, 0.0, 0.0, 1.0]])
    H_cam2_cam1 = np.array([[0.9999714283197217, -0.0006159292736051988, 0.007534134014844326, -0.10245440610370138],
                            [0.0006119760079932377, 0.9999996738738459, 0.0005270081360818972, 0.0001138693144027686],
                            [-0.007534456157504587, -0.0005223823693158363, 0.9999714791368196, 0.0028387840213753655],
                            [0.0, 0.0, 0.0, 1.0]])

    # ======================================================================================
    # Transform the point from rgb to cam 1 frame
    # ======================================================================================
    H_cam_1_point = np.matmul(H_cam1_rgb, H_rgb_2_point)
    t_cam1_2_point = H_cam_1_point[:3, 3]
    # cam1 parameters
    # camera_mtx_cam1 = np.array(
    #    [[704.7619734668378, 0, 300.78994093351923], [0, 705.9081489448346, 229.8585626133938], [0, 0, 1]])
    # distortion_coeffs_cam1 = np.array(
    #    [-0.364714404546865, 0.07816135096611694, 0.005085888890347796, -0.0036418328217707836])

    # Project 3D points to image plane
    # points_2d_cam1, _ = cv2.projectPoints(np.array([t_cam1_2_point]), np.array([[0, 0, 0]]), np.array([[0, 0, 0]]),
    #                                     camera_mtx_cam1, distortion_coeffs_cam1)
    # points_2d_cam1 = np.round(points_2d_cam1[0]).astype(int)
    # print(points_2d_cam1)
    # Display the 2d points on the image
    # img_test = cv2.circle(image_left, tuple(points_2d_cam1[0][0]), 10, (255, 0, 0), -1)
    # cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))  # resize image to 0.5 for display
    # cv2.waitKey(0)

    # ======================================================================================
    # Transform the point from cam 1 to cam 2 frame
    # ======================================================================================
    H_cam_2_point = np.matmul(H_cam2_cam1, H_cam_1_point)
    t_cam2_2_point = H_cam_2_point[:3, 3]
    # cam2 Parameters
    # camera_mtx_cam2 = np.array(
    #    [[2.6174837933891757, 0, 2644.449052050593], [0, 2655.684777520004, 308.2314261541802], [0, 0, 1]])
    # distortion_coeffs_cam2 = np.array(
    #    [0.4008405051407828, -35.782378911149834, 0.018867056305151703, -0.0068405689007467185])

    # Project 3D points to image plane
    # points_2d_cam2, _ = cv2.projectPoints(np.array([t_cam2_2_point]), np.array([[0, 0, 0]]), np.array([[0, 0, 0]]),
    #                                      camera_mtx_cam2, distortion_coeffs_cam2)
    # points_2d_cam2 = np.round(points_2d_cam2[0]).astype(int)
    # print(points_2d_cam2)
    # Display the 2d points on the image
    # img_test = cv2.circle(image_right, tuple(points_2d_cam2[0][0]), 10, (255, 0, 0), -1)
    # cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))  # resize image to 0.5 for display
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    with open(json_file_vicon_path, 'w') as json_file:
        json.dump(vicon_data, json_file)
    with open(json_file_event_left_path, 'w') as json_file:
        json.dump(t_cam1_2_point, json_file)  #
    with open(json_file_event_right_path, 'w') as json_file:
        json.dump(t_cam2_2_point, json_file)
    with open(json_file_rgb_path, 'w') as json_file:
        json.dump(t_cam_optical_2_point, json_file)


# Folder to save images
save_folder = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration'
os.makedirs(save_folder, exist_ok=True)
image_dir = os.path.join(save_folder, 'images')
os.makedirs(image_dir, exist_ok=True)
# json_file_path = os.path.join(save_folder, 'vicon_coordinates.json')

event_callback_left.counter = 0
flag = 0
vicon_data = {}
rate = rospy.Rate(5)
#callback_thread_instance = Thread(target=callback_thread_left)
#callback_thread_instance.start()
rospy.Subscriber(event_topic_left, EventArray, event_callback_left)
rospy.Subscriber(event_topic_right, EventArray, event_callback_right)
image_subscriber = rospy.Subscriber('/rgb/image_raw', Image, image_callback)
rospy.Subscriber('/vicon/event_cam_sys/event_cam_sys', TransformStamped, vicon_callback)
#rospy.Subscriber('/vicon/markers', TransformStamped, vicon_obj_callback)
#callback_thread_instance2 = Thread(target=callback_thread_right)
#callback_thread_instance2.start()
#callback_thread_instance3 = Thread(target=callback_thread_image)
#callback_thread_instance3.start()
#callback_thread_instance4 = Thread(target=callback_thread_vicon)
#callback_thread_instance4.start()

while not rospy.is_shutdown():
    print('inside while loop')
    #print(vicon_cam_rotation)
    print(flag_vicon, flag_rgb, flag_event_left, flag_event_right)
    if flag_event_left == 1 and flag_event_right == 1 and flag_rgb == 1 and flag_vicon == 1:
        print('inside if')
        #compute_transformation(event_left, event_right, vicon_cam_translation, vicon_cam_rotation, received_image)
        print("Computing transformations after receiving 5 messages")
        for event_traverser in range(len(event_left)):
            x_left[event_traverser] = event_left[event_traverser].x
            y_left[event_traverser] = event_left[event_traverser].y
            polarity_left[event_traverser] = event_left[event_traverser].polarity
            image_left[event_left[event_traverser].y, event_left[event_traverser].x] = int(
                event_left.events[event_traverser].polarity) * 254
            # visualize image using opencv
            cv2.imshow('image', image_left)
            cv2.waitKey(1)
            # cv2.destroyAllWindows()
        for event_traverser in range(len(event_right)):
            x_right[event_traverser] = event_right[event_traverser].x
            y_right[event_traverser] = event_right[event_traverser].y
            polarity_right[event_traverser] = event_right[event_traverser].polarity
            image_right[event_right[event_traverser].y, event_right[event_traverser].x] = int(
                event_right.events[event_traverser].polarity) * 254
            # visualize image using opencv
            cv2.imshow('image', image_right)
            cv2.waitKey(1)
            # cv2.destroyAllWindows()

        H_cam_vicon_2_cam_optical = np.array([
            [0.1844511, 0.10184152, 0.97755107, 0.13975843],
            [-0.98279335, 0.02897629, 0.18242149, -0.54963646],
            [-0.00974772, -0.99437854, 0.10543387, -0.50285077],
            [0., 0., 0., 1.]])

        params = [1.93686226e+03, 1.93330361e+03, 9.85688080e+02, 7.71096964e+02]
        camera_matrix = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
        distortion_coefficients = np.array([-0.12905658, -0.01019267, 0.00562304, -0.00015354, 0.13542021])

        # base to camera_vicon transformation. The below values are taken from recorded json file
        # get rotation matrix from quaternion
        rotation = R.from_quat(vicon_cam_rotation).as_matrix()
        # make homogeneous transformation matrix
        H_base_2_cam_vicon = np.eye(4)
        H_base_2_cam_vicon[:3, :3] = rotation
        H_base_2_cam_vicon[:3, 3] = vicon_cam_translation

        # make homogeneous transformation matrix from base to camera optical frame
        H_base_2_cam_optical = np.matmul(H_base_2_cam_vicon, H_cam_vicon_2_cam_optical)

        # invert H_vicon_2_cam_optical to get H_cam_optical_2_vicon
        H_cam_optical_2_vicon = np.eye(4)
        H_cam_optical_2_vicon[:3, :3] = np.transpose(H_base_2_cam_optical[:3, :3])
        H_cam_optical_2_vicon[:3, 3] = -np.matmul(np.transpose(H_base_2_cam_optical[:3, :3]), H_base_2_cam_optical[:3, 3])

        # TODO: get the translation and rotation vec from the moving object by subscribing to the vicon topic
        H_v_2_point = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # transform the point to the camera optical frame
        H_cam_optical_2_point = np.matmul(H_cam_optical_2_vicon, H_v_2_point)

        # get the 3d point
        t_cam_optical_2_point = H_cam_optical_2_point[:3, 3]
        t_rgb_2_point = t_cam_optical_2_point
        H_rgb_2_point = H_cam_optical_2_point

        H_cam1_rgb = np.array([[0.9966050852350032, 0.0018025810956462127, 0.08231072096103041, 0.058348558249773974],
                               [-0.00029275262105358537, 0.9998315556310521, -0.018351421206337717, 0.0015723475769044038],
                               [-0.0823299361085253, 0.018265023016223507, 0.9964377404306713, 0.02935723981670002],
                               [0.0, 0.0, 0.0, 1.0]])
        H_cam2_cam1 = np.array([[0.9999714283197217, -0.0006159292736051988, 0.007534134014844326, -0.10245440610370138],
                                [0.0006119760079932377, 0.9999996738738459, 0.0005270081360818972, 0.0001138693144027686],
                                [-0.007534456157504587, -0.0005223823693158363, 0.9999714791368196, 0.0028387840213753655],
                                [0.0, 0.0, 0.0, 1.0]])

        # ======================================================================================
        # Transform the point from rgb to cam 1 frame
        # ======================================================================================
        H_cam_1_point = np.matmul(H_cam1_rgb, H_rgb_2_point)
        t_cam1_2_point = H_cam_1_point[:3, 3]
        # cam1 parameters
        # camera_mtx_cam1 = np.array(
        #    [[704.7619734668378, 0, 300.78994093351923], [0, 705.9081489448346, 229.8585626133938], [0, 0, 1]])
        # distortion_coeffs_cam1 = np.array(
        #    [-0.364714404546865, 0.07816135096611694, 0.005085888890347796, -0.0036418328217707836])

        # Project 3D points to image plane
        # points_2d_cam1, _ = cv2.projectPoints(np.array([t_cam1_2_point]), np.array([[0, 0, 0]]), np.array([[0, 0, 0]]),
        #                                     camera_mtx_cam1, distortion_coeffs_cam1)
        # points_2d_cam1 = np.round(points_2d_cam1[0]).astype(int)
        # print(points_2d_cam1)
        # Display the 2d points on the image
        # img_test = cv2.circle(image_left, tuple(points_2d_cam1[0][0]), 10, (255, 0, 0), -1)
        # cv2.imshow('img', cv2.resize(img_test, (0, 0), fx=0.5, fy=0.5))  # resize image to 0.5 for display
        # cv2.waitKey(0)

        # ======================================================================================
        # Transform the point from cam 1 to cam 2 frame
        # ======================================================================================
        H_cam_2_point = np.matmul(H_cam2_cam1, H_cam_1_point)
        t_cam2_2_point = H_cam_2_point[:3, 3]
        rate.sleep()