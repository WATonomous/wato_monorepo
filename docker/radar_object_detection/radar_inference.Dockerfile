FROM nvidia/cuda:11.3.1-devel-ubuntu20.04 

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

# # Add a docker user so we that created files in the docker container are owned by a non-root user
# RUN addgroup --gid 1000 docker && \
#     adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
#     echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# # Remap the docker user and group to be the same uid and group as the host user.
# # Any created files by the docker container will be owned by the host user.
# RUN USER=docker && \
#     GROUP=docker && \
#     curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
#     chown root:root /usr/local/bin/fixuid && \
#     chmod 4755 /usr/local/bin/fixuid && \
#     mkdir -p /etc/fixuid && \
#     printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml

# USER docker:docker

RUN apt-get update && apt-get install wget -yq
RUN apt-get install build-essential g++ gcc -y
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install libgl1-mesa-glx libglib2.0-0 -y
RUN apt-get install openmpi-bin openmpi-common libopenmpi-dev libgtk2.0-dev git -y

# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH
RUN conda install python=3.8
RUN conda install pytorch==1.10.1 torchvision==0.11.2 torchaudio==0.10.1 cudatoolkit=11.3 -c pytorch
RUN pip install Pillow==8.4.0
RUN pip install tqdm
RUN pip install torchpack
RUN pip install mmcv==1.4.0 mmcv-full==1.4.0 mmdet==2.20.0
RUN pip install nuscenes-devkit
RUN pip install mpi4py==3.0.3
RUN pip install numba==0.48.0

WORKDIR /mnt
COPY src/testing_pretrained testing_pretrained

WORKDIR /