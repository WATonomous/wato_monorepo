ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for world modeling build
COPY src/world_modeling world_modeling
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Keep engine-builder packages separate from the ROS Python environment.
# Copy only metadata so changes to the eval scripts do not reinstall TensorRT.
ENV MTR_ENGINE_VENV=/opt/watonomous/mtr-engine-venv
COPY src/world_modeling/prediction_ml/eval/pyproject.toml /tmp/mtr-pyproject.toml
RUN python3 -m venv "${MTR_ENGINE_VENV}" && \
    "${MTR_ENGINE_VENV}/bin/python" -c \
      'import tomllib; p = tomllib.load(open("/tmp/mtr-pyproject.toml", "rb"))["project"]; print(*p["dependencies"], *p["optional-dependencies"]["engine"], sep="\n")' \
      > /tmp/mtr-engine-requirements.txt && \
    "${MTR_ENGINE_VENV}/bin/python" -m pip install --no-cache-dir -r /tmp/mtr-engine-requirements.txt && \
    "${MTR_ENGINE_VENV}/bin/python" -c 'import onnx, tensorrt, yaml; from cuda.bindings import runtime' && \
    rm -f /tmp/mtr-pyproject.toml /tmp/mtr-engine-requirements.txt

# The TensorRT pip wheels ship libraries but no C++ headers. Add the headers for the exact
# installed release so prediction_ml can compile its TensorRT backend against this venv.
# The CUDA runtime wheel's cuda_runtime_api.h needs crt/ headers, which only the nvcc wheel of the
# same CUDA release provides.
RUN CUDA_MM="$("${MTR_ENGINE_VENV}/bin/python" -c 'import importlib.metadata as m; print(".".join(m.version("nvidia-cuda-runtime-cu12").split(".")[:2]))')" && \
    "${MTR_ENGINE_VENV}/bin/python" -m pip install --no-cache-dir "nvidia-cuda-nvcc-cu12==${CUDA_MM}.*" && \
    TRT_TAG="v$("${MTR_ENGINE_VENV}/bin/python" -c 'import tensorrt; print(".".join(tensorrt.__version__.split(".")[:3]))')" && \
    git -c advice.detachedHead=false clone --quiet --depth 1 --branch "${TRT_TAG}" \
      --filter=blob:none --sparse https://github.com/NVIDIA/TensorRT /tmp/tensorrt-src && \
    git -C /tmp/tensorrt-src sparse-checkout set include && \
    mkdir -p "${MTR_ENGINE_VENV}/tensorrt" && \
    cp -r /tmp/tensorrt-src/include "${MTR_ENGINE_VENV}/tensorrt/include" && \
    rm -rf /tmp/tensorrt-src && \
    test -f "${MTR_ENGINE_VENV}/tensorrt/include/NvInfer.h"
