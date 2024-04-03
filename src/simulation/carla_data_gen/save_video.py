import cv2
import numpy as np
import os
from glob import glob

# Directory containing images
image_folder = 'images'
# Output video file
# video_file = 'sim_video.mp4'
video_file = 'sim_video.mp4'

# images = sorted(glob(f"{image_folder}/*_camera.png"))
images = sorted(glob(f"{image_folder}/*_bbox.png"))

# Determine the width and height from the first image
image_path = images[0]
frame = cv2.imread(image_path)
height, width, layers = frame.shape

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use lower case
out = cv2.VideoWriter(video_file, fourcc, 20.0, (width, height))

for i,image in enumerate(images):
    img = cv2.imread(image)
    print(i, "out of", len(images))
    out.write(img)  # Write out frame to video

out.release()  # Release the video writer

# Check if video file has been created and return appropriate response
if os.path.exists(video_file):
    video_file_path = video_file
else:
    video_file_path = "Video conversion failed!"

video_file_path
