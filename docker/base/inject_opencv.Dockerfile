ARG GENERIC_IMAGE

########################## Install ROS2 Core ##########################
FROM ${GENERIC_IMAGE} as wato_base

RUN apt-get update
RUN apt install -y libopencv-dev python3-opencv
RUN export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake