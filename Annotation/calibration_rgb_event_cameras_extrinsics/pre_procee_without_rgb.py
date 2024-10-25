import rosbag
import json
import os

count = 0
json_file_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/calib_23_oct/scene_1/vicon_coordinates.json'
vicon_data = {}
iterator = 0
# read rosbag file
bag = rosbag.Bag('/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/calib_23_oct/scene_1/scene_1.bag')
vicon_topic_cam_sys = '/vicon/event_cam_sys/event_cam_sys'

for top, msg, tim in bag.read_messages(vicon_topic_cam_sys):
    # rea time in nano secs
    # save every 100th sample to reduce the size of the data
    if count % 100 == 0:
        timestamp = msg.header.stamp.to_nsec()
        translation = [
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z]
        rotation = [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ]
        vicon_data[iterator] = {'timestamp': timestamp, 'translation': translation, 'rotation': rotation}
        iterator += 1
    count += 1
with open(json_file_path, 'w') as json_file:
    json.dump(vicon_data, json_file)
print("done")

image_path = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/calib_23_oct/scene_1/event_cam_left/e2calib'
dest = '/home/eventcamera/Eventcamera/vicon_rgb_extrinsic_calibration/calib_23_oct/scene_1/images/'
# list images
image_list = os.listdir(image_path)
count = 0
#save every 100 image to dest
for i in range(0, len(image_list), 100):
    os.rename(image_path + '/' + image_list[i], dest + str(count) + '.png')
    count += 1