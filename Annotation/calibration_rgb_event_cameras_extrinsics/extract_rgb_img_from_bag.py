

import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

output_dir = '/media/eventcamera/event_data/calibration/mar_20/cam1/'
BAGFILE = '/media/eventcamera/event_data/calibration/mar_20/rgb.bag'
#os.mkdir(output_dir)

# if cam0 not there then create it
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

if __name__ == '__main__':
    bag = rosbag.Bag(BAGFILE)
    TOPIC = '/camera/image_raw'
    DESCRIPTION = 'color_'
    image_topic = bag.read_messages(TOPIC)
    i = 0
    for k, b in enumerate(image_topic):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(b.message, "bgr8")
        #cv_image.astype(np.uint8)


        #cv_image = cv_image[45:480,0:595]
        #cv_image = cv2.resize(cv_image, (640,480))
        cv2.imwrite(output_dir + str(b.timestamp) + '.png', cv_image)
        print('saved: ',i)
        i += 1


bag.close()

print('PROCESS COMPLETE')
