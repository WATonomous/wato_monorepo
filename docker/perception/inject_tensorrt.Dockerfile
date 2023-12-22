ARG GENERIC_IMAGE
ARG TENSORRT_PKG

########################## Install TensorRT ##########################
FROM {GENERIC_IMAGE} as core

# List of TensorRT Packages (must login): 
# https://developer.nvidia.com/tensorrt-download
ENV PKG=${TENSORRT_PKG}

RUN dpkg -i ${PKG}.deb && \
    cp /var/${PKG%_*_*}/*-keyring.gpg /usr/share/keyrings/

# Installs full runtime (lean runtimes also available: 
# https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html)
RUN apt-get update && apt-get q -y --no-install-recommends \
    tensorrt
