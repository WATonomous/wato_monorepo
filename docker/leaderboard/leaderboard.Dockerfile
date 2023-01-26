FROM ros:foxy

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

COPY src/leaderboard/.bashrc /homer/docker/
