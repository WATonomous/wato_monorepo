# ================= Dependencies ===================
FROM leungjch/cuda118-tensorrt-base as base
# Segmentation Dependencies
# Install essential packages and dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    git \
    make build-essential libssl-dev zlib1g-dev \
    libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
    libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev \
    ffmpeg libsm6 libxext6 cmake libgl1-mesa-glx \
    && sudo rm -rf /var/lib/apt/lists/*

RUN curl https://pyenv.run | bash

ENV PATH="/root/.pyenv/bin:$PATH"
ENV PYENV_ROOT="/root/.pyenv"

SHELL ["/bin/bash", "-c"]
# RUN source "$HOME/.bashrc" && pyenv install 3.8.15 && pyenv global 3.8.15
SHELL ["/bin/sh", "-c"]

RUN sudo apt-get install -y python3-pip

RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel
RUN pip3 install ninja
RUN pip install --no-cache-dir --upgrade requests urllib3


ENV DEBIAN_FRONTEND noninteractive
# RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

# RUN chmod +r /src/perception/camera_segmentation/camera_segmentation/data/bpe_simple_vocab_16e6.txt

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

COPY src/perception/camera_segmentation/ camera_segmentation
COPY src/wato_msgs/sample_msgs sample_msgs

# RUN sudo apt-get install ros-humble-cv-bridge
COPY src/perception/camera_segmentation/requirements.txt camera_segmentation/requirements.txt
RUN pip3 install --ignore-installed PyYAML
RUN pip3 install --ignore-installed --no-cache-dir --upgrade --trusted-host pypi.org --trusted-host shi-labs.com --trusted-host files.pythonhosted.org -r camera_segmentation/requirements.txt

WORKDIR /home/docker/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src \
    --rosdistro $ROS_DISTRO -y  && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/wato_ros_entrypoint.sh /ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc

RUN chmod -R 777 /home/docker
RUN sudo mkdir -p -m 777 /.ros
RUN chmod -R 777 /.ros
USER docker:docker

ENTRYPOINT ["sh", "/home/docker/wato_ros_entrypoint.sh"]
CMD ["/bin/bash", "/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/run.sh"]