# Stage 1: get wato-cuda-bevfusion at the same commit as docker/perception.Dockerfile,
# so training, export and the deployed node use one version.
FROM alpine/git AS source
# Keep this in sync with docker/perception.Dockerfile.
ARG CUDA_BEVFUSION_COMMIT=08ec4db80ec768e9410c11e032e2a26dd24ded52
# Alpine package versions change with each release.
# hadolint ignore=DL3018
RUN apk add --no-cache git-lfs && \
    git clone https://github.com/WATonomous/wato-cuda-bevfusion.git /src/wato-cuda-bevfusion && \
    git -C /src/wato-cuda-bevfusion checkout "${CUDA_BEVFUSION_COMMIT}" && \
    git -C /src/wato-cuda-bevfusion submodule update --init --recursive && \
    git -C /src/wato-cuda-bevfusion lfs install && \
    git -C /src/wato-cuda-bevfusion lfs pull

# Stage 2: training environment (Python 3.8, PyTorch 1.10, mmcv 1.4.0, CUDA 11.x devel)
FROM bevfusion:upstream
COPY --from=source /src/wato-cuda-bevfusion /opt/wato-cuda-bevfusion
ENV CB=/opt/wato-cuda-bevfusion/CUDA-BEVFusion
ENV BF=${CB}/bevfusion
# ONNX and quantization tools used by the qat/ export scripts
RUN sed -e '/pypi.ngc.nvidia.com/d' \
        -e 's/^pytorch-quantization.*/pytorch-quantization==2.1.2/' \
        ${CB}/tool/requirements.txt > /tmp/qat-requirements.txt && \
    cat /tmp/qat-requirements.txt && \
    pip install --no-cache-dir --extra-index-url https://pypi.nvidia.com -r /tmp/qat-requirements.txt
# Compile BEVFusion's CUDA ops at build time.
# No GPU is visible during docker build, so setup.py would skip CUDA unless FORCE_CUDA=1.
# GPU code is compiled for these compute capabilities: 8.0 = A100, 8.6 = RTX 30xx / A40 / A6000.
# +PTX lets newer GPUs (for example, RTX 40xx) compile it at first run.
# For older GPUs, pass e.g. --build-arg TORCH_CUDA_ARCH_LIST="7.5;8.0;8.6+PTX" (7.5 = RTX 20xx / T4).
ARG TORCH_CUDA_ARCH_LIST="8.0;8.6+PTX"
ENV TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_LIST}
ENV FORCE_CUDA=1
# ninja makes the extension build parallel; MAX_JOBS caps memory use during compilation
RUN pip install --no-cache-dir ninja==1.11.1
ENV MAX_JOBS=8
WORKDIR ${BF}
RUN python setup.py develop
