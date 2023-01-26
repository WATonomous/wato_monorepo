FROM ros:foxy as base

RUN apt-get update && apt-get install -y wget git tar

WORKDIR /home/docker
RUN wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Leaderboard/CARLA_Leaderboard_20.tar.gz && \
    tar -xf CARLA_Leaderboard_20.tar.gz

# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH
# RUN conda init bash && \
#     conda create -n py37 python=3.7 && \
#     conda activate py37
RUN conda install python=3.7
RUN pip3 install -r PythonAPI/carla/requirements.txt

RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/leaderboard.git
RUN cd leaderboard && pip3 install -r requirements.txt

RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git
RUN cd scenario_runner && pip3 install -r requirements.txt

RUN git clone --recurse-submodules -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/ros-bridge

RUN apt-get update && apt-get install -y libomp5 curl
# Add a docker user so that created files in the docker container are owned by a non-root user
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd
# Remap the docker user and group to be the same uid and group as the host user.
# Any created files by the docker container will be owned by the host user.
RUN USER=docker && \
    GROUP=docker && \                                                                     
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \                                                                                                            
    chown root:root /usr/local/bin/fixuid && \                                                                              
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \                                                                                               
    printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml
USER docker:docker

COPY src/leaderboard/.bashrc /homer/docker/
