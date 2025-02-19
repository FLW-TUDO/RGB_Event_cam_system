import rosbag
import numpy as np
import os
import cv2
import threading
import queue

object_name = 'scene_zivid_1'
bag = rosbag.Bag('/media/eventcamera/event_data/' + object_name + '/right.bag')
path = '/media/eventcamera/event_data/' + object_name + '/'

# Event camera topics
event_topics = {
    "left": "/dvxplorer_left/events",
    "right": "/dvxplorer_right/events"
}

# Output directory for images
output_dir = path + "event_images"
os.makedirs(output_dir, exist_ok=True)
os.makedirs(os.path.join(output_dir, "left"), exist_ok=True)
os.makedirs(os.path.join(output_dir, "right"), exist_ok=True)

# Time step for slicing (in nanoseconds) - Example: 10ms
time_step_ns = 10_000_000

# Check for GPU acceleration (CUDA support)
try:
    cv2.ocl.setUseOpenCL(True)
    use_gpu = cv2.cuda.getCudaEnabledDeviceCount() > 0
    print(f"CUDA enabled: {use_gpu}")
except:
    use_gpu = False

# Event queues for both cameras
event_queues = {
    "left": queue.Queue(maxsize=10),
    "right": queue.Queue(maxsize=10)
}

# Function to process events and generate images for a given camera
def process_events(camera_name):
    while True:
        batch = event_queues[camera_name].get()
        if batch is None:
            break

        batch_timestamp, events = batch  # Get assigned timestamp and events
        img = np.zeros((480, 640), dtype=np.uint8)  # Blank grayscale image

        # Use GPU if available
        if use_gpu:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(img)

            for x, y, polarity in events:
                color = 255 if polarity else 100  # White for positive, Gray for negative
                if 0 <= x < 640 and 0 <= y < 480:
                    img[y, x] = color  # Draw event

            gpu_img.download(img)  # Copy processed image back to CPU memory
        else:
            for x, y, polarity in events:
                color = 255 if polarity else 100
                if 0 <= x < 640 and 0 <= y < 480:
                    img[y, x] = color

        # Save image using a clean timestamp format (integer nanoseconds)
        image_filename = os.path.join(output_dir, camera_name, f"{int(batch_timestamp)}.png")
        cv2.imwrite(image_filename, img)
        print(f"Saved {image_filename} with {len(events)} events.")

# Start processing threads for both cameras
processing_threads = {}
for cam in event_topics.keys():
    processing_threads[cam] = threading.Thread(target=process_events, args=(cam,))
    processing_threads[cam].start()


# Track current batches and time windows for both cameras
current_batches = {"left": [], "right": []}
current_time_windows = {"left": None, "right": None}

# Read messages from both topics
for topic, msg, t in bag.read_messages(topics=event_topics.values()):
    camera_name = "left" if topic == event_topics["left"] else "right"

    for event in msg.events:
        event_time_ns = event.ts.to_nsec()  # Get event timestamp in nanoseconds

        # Initialize time window on first event
        if current_time_windows[camera_name] is None:
            current_time_windows[camera_name] = (event_time_ns // time_step_ns) * time_step_ns

        # If the event belongs to the current time window, add it to the batch
        if event_time_ns < current_time_windows[camera_name] + time_step_ns:
            current_batches[camera_name].append((event.x, event.y, event.polarity))
        else:
            # Send previous batch for processing
            if current_batches[camera_name]:
                event_queues[camera_name].put((current_time_windows[camera_name], current_batches[camera_name]))

            # Move to the next time window
            current_time_windows[camera_name] = (event_time_ns // time_step_ns) * time_step_ns
            current_batches[camera_name] = [(event.x, event.y, event.polarity)]  # Start new batch

# Process remaining events
for cam in event_topics.keys():
    if current_batches[cam]:
        event_queues[cam].put((current_time_windows[cam], current_batches[cam]))

# Signal processing threads to stop
for cam in event_topics.keys():
    event_queues[cam].put(None)

# Wait for threads to finish
for cam in processing_threads.keys():
    processing_threads[cam].join()

# Close the ROS bag
bag.close()
print("Event visualization completed for both cameras.")
