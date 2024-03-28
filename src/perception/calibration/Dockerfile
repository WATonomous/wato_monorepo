FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt update && apt install -y ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-opencv
RUN pip3 install opencv-python-headless 
WORKDIR /app

COPY *.py .
COPY *.jpg .

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash &&  python3 calibrate_ros.py"]
