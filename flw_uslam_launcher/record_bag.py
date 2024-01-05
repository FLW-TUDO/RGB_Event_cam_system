import numpy as np
import cv2
import os

import rospy
import rosbag

from sensor_msgs.msg import Image
from dvs_msgs.msg import EventArray

from cv_bridge import CvBridge

# ______________________________________________
# Parameters
# ______________________________________________
save_path = "/home/gouda/uslam_ws/src/misc/"
# ______________________________________________

rgb_topic = '/cam1/image_raw'
event_topic = '/dvs/events'


i = 0
def event_callback(event_data):
    bag.write(event_topic, event_data)


def rgb_callback(rgb_data):
    global i
    i = i + 1
    cv_image = bridge.imgmsg_to_cv2(rgb_data, desired_encoding='passthrough')
    rgb_timestamp = int(rgb_data.header.stamp.nsecs/1000) + rgb_data.header.stamp.secs*1000000
    rgb_timestamp *= 1000
    cv2.imwrite(cam0_dir + '/' + str(rgb_timestamp) + '.png', cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
    #cv2.imwrite(cam0_dir + '/' + str(i) + '.png', cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

if __name__ == '__main__':
    
    rospy.init_node('listener', anonymous=True)

    rgb_sub = rospy.Subscriber(rgb_topic, Image, rgb_callback)
    event_sub = rospy.Subscriber(event_topic, EventArray, event_callback)

    bag = rosbag.Bag(save_path + '/events.bag', 'w')

    bridge = CvBridge()

    cam0_dir = save_path + '/_cam0_1280'
    os.mkdir(cam0_dir)

    rospy.spin()

    event_sub.unregister()
    rgb_sub.unregister()
    bag.close()

    print("Finished Collection")

