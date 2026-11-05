# Stage 1: clone wato-cuda-bevfusion the same way docker/perception.Dockerfile does.
# TODO: unpinned (latest main). See the warning at the top of the work-split doc.
FROM alpine/git AS source
RUN apk add --no-cache git-lfs && \
    git clone --recurse-submodules https://github.com/WATonomous/wato-cuda-bevfusion.git /src/wato-cuda-bevfusion && \
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
# Compile BEVFusion's CUDA ops at build time, for A100 and RTX 3090.
# No GPU is visible during docker build, so setup.py would skip CUDA unless FORCE_CUDA=1.
ENV TORCH_CUDA_ARCH_LIST="8.0;8.6"
ENV FORCE_CUDA=1
# ninja makes the extension build parallel; MAX_JOBS caps memory use during compilation
RUN pip install --no-cache-dir ninja
ENV MAX_JOBS=8
RUN cd ${BF} && python setup.py develop
