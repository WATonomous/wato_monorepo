# Calibration image: koide3/direct_visual_lidar_calibration (ROS 2 Jazzy, reads the
# car's MCAP bags) plus the Python dependencies of lidar_cam_calib's scripts.
FROM koide3/direct_visual_lidar_calibration:jazzy

# xvfb: calibrate opens an OpenGL viewer even in batch runs; xacro: initial guess from the URDF
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-numpy python3-scipy python3-opencv python3-yaml \
        ros-jazzy-xacro ros-jazzy-sensor-msgs-py ros-jazzy-rosbag2-storage-mcap \
        xvfb xauth \
    && rm -rf /var/lib/apt/lists/*

# small_gicp's Python bindings estimate the car's motion from the LiDAR
RUN pip3 install --no-cache-dir --break-system-packages small_gicp==1.0.1

WORKDIR /ws
