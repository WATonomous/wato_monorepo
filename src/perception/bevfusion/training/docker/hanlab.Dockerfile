# Base image with the library versions upstream BEVFusion needs (Python 3.8, PyTorch 1.10.1 + CUDA 11.3,
# mmcv-full 1.4.0, mmdet 2.20.0). Build it as bevfusion:upstream; docker/Dockerfile builds on it.
FROM nvidia/cuda:11.3.1-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
# Recommended packages are kept to match the original upstream image.
# hadolint ignore=DL3015
RUN apt-get update && apt-get install -y wget build-essential g++ gcc libgl1-mesa-glx libglib2.0-0 openmpi-bin openmpi-common libopenmpi-dev libgtk2.0-dev git && \
    rm -rf /var/lib/apt/lists/*

# Install miniconda (Python 3.8 build, so no `conda install python=3.8` and no conda ToS prompt)
ENV CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-py38_23.11.0-2-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda
# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH
# Unpinned packages match upstream BEVFusion's README.
# hadolint ignore=DL3013
RUN pip install --no-cache-dir torch==1.10.1+cu113 torchvision==0.11.2+cu113 torchaudio==0.10.1+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html && \
    pip install --no-cache-dir Pillow==8.4.0 && \
    pip install --no-cache-dir tqdm && \
    pip install --no-cache-dir torchpack && \
    pip install --no-cache-dir mmcv-full==1.4.0 -f https://download.openmmlab.com/mmcv/dist/cu113/torch1.10.0/index.html && \
    pip install --no-cache-dir mmdet==2.20.0 && \
    pip install --no-cache-dir nuscenes-devkit && \
    pip install --no-cache-dir mpi4py==3.0.3 && \
    pip install --no-cache-dir numba==0.48.0
