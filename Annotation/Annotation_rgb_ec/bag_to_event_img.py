import rosbag
import numpy as np
import os
import cv2
import threading
import queue

objects = ['MR6D1','MR6D2','MR6D3']
object_name = 'scene_12'
bag = rosbag.Bag('//media/eventcamera/Windows/dataset_7_feb/' + object_name + '/' + object_name + '.bag')
path = '//media/eventcamera/Windows/dataset_7_feb/' + object_name + '/'
events_topic_left = '/dvxplorer_left/events'


# Output directory for images
output_dir = "//media/eventcamera/Windows/dataset_7_feb/" + object_name + "/event_images"
os.makedirs(output_dir, exist_ok=True)

# Open the ROS bag file
#bag = rosbag.Bag(bag_file, "r")

# Initialize batch processing
batch_size = 15000  # Process 10,000 events at a time
current_batch = []
batch_timestamp = None
event_queue = queue.Queue(maxsize=10)
try:
    cv2.ocl.setUseOpenCL(True)
    use_gpu = cv2.cuda.getCudaEnabledDeviceCount() > 0
    print(f"CUDA enabled: {use_gpu}")
except:
    use_gpu = False

# Function to visualize and save batches
def process_events():
    while True:
        batch = event_queue.get()  # Get the next batch
        if batch is None:
            break  # Stop processing when no more data

        batch_timestamp, events = batch
        img = np.zeros((480, 640), dtype=np.uint8)  # Create a blank image

        # If GPU is available, use OpenCV CUDA
        if use_gpu:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(img)

            for x, y, polarity in events:
                color = 255 if polarity else 100  # White for positive, Gray for negative
                if 0 <= x < 640 and 0 <= y < 480:
                    img[y, x] = color  # Draw event

            gpu_img.download(img)  # Transfer back to CPU memory
        else:
            for x, y, polarity in events:
                color = 255 if polarity else 100
                if 0 <= x < 640 and 0 <= y < 480:
                    img[y, x] = color

        # Save image with timestamp filename
        image_filename = os.path.join(output_dir, f"{int(batch_timestamp)}.png")
        cv2.imwrite(image_filename, img)
        print(f"Saved {image_filename} with {len(events)} events.")

# Start the processing thread
processing_thread = threading.Thread(target=process_events)
processing_thread.start()

for topic, msg, t in bag.read_messages(topics=[events_topic_left]):
    start_time = msg.events[0].ts.to_nsec()  # First event timestamp in nanoseconds
    end_time = msg.events[-1].ts.to_nsec()  # Last event timestamp in nanoseconds
    total_events = len(msg.events)

    # Determine the time step for each batch
    if total_events >= batch_size:
        time_step = (end_time - start_time) / (total_events // batch_size)
    else:
        time_step = (end_time - start_time) / total_events

    batch_timestamp = start_time  # Initialize batch timestamp

    for event in msg.events:
        current_batch.append((event.x, event.y, event.polarity))  # Store event data

        if len(current_batch) >= batch_size:
            event_queue.put((batch_timestamp, current_batch))  # Assign timestamp
            batch_timestamp += time_step * batch_size  # Move to next time step
            current_batch = []  # Reset batch

# Push any remaining events
if current_batch:
    event_queue.put((batch_timestamp, current_batch))

# Signal processing thread to stop
event_queue.put(None)
processing_thread.join()

# Close the ROS bag
bag.close()
print("Event visualization completed.")