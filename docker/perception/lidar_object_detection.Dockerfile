# FROM leungjch/cuda118-tensorrt-base:latest as base
FROM leungjch/cuda113-tensorrt-base:latest


RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# RUN sudo sed -i \
#     's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
#     /ros_entrypoint.sh && \
#     cat /ros_entrypoint.sh

RUN nvidia-smi >&2
RUN rm -rf /etc/ros/rosdep/sources.list.d/

RUN ROS_DISTRO=humble

RUN apt-get update && apt install -y tzdata
RUN apt-get install -y python3.8 python3-pip git libgl1-mesa-glx libglib2.0-0

RUN pip install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
RUN pip install pyyaml scikit-image onnx onnx-simplifier spconv==2.3.6 pillow==10.0.0 numba==0.58.0 opencv-python
RUN pip install onnx_graphsurgeon --index-url https://pypi.ngc.nvidia.com

# # convert the pointpillars model from etlt format to tensorrt engine   
# RUN /model/pointpillars_model/tao-converter  -k tlt_encode \
#                -e /model/pointpillars_model/trt.engine \
#                -p points,1x204800x4,1x204800x4,1x204800x4 \
#                -p num_points,1,1,1 \
#                -t fp16 \
#                /model/pointpillars_model/pointpillars_deployable.etlt
# USER docker:docker

RUN mkdir -p /home/docker/ament_ws
WORKDIR /home/docker/ament_ws
COPY src/perception/lidar_object_detection/model /model/pointpillars_model
COPY src/perception/lidar_object_detection pointpillars_ws

# Get environment variables
# RUN . pointpillars_ws/lidar_object_detection_node/src/CUDA-PointPillars/tool/environment.sh

RUN export DEBIAN_FRONTEND=noninteractive && . /opt/ros/$ROS_DISTRO/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path pointpillars_ws \
    --rosdistro $ROS_DISTRO -y \
    --skip-keys "rti-connext-dds-6.0.1" && \
    colcon build \
        --merge-install \
        --packages-select pp_infer \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.3

# Entrypoint will run before any CMD on launch. Sources ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x /home/docker/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws

# ENTRYPOINT ["tail", "-f", "/dev/null"]

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["ros2", "launch", "pp_infer", "pp_infer_launch.py"]
# Must run with DOCKER_BUILDKIT=0 ./watod up --build
# Buildkit does not support gpus as of Sept17 https://github.com/docker/compose/issues/9681