FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates git python3-venv python3-yaml python3-colcon-common-extensions libvulkan1 libegl1 libgl1 \
    ros-jazzy-ackermann-msgs ros-jazzy-nav-msgs ros-jazzy-geometry-msgs \
    ros-jazzy-rosgraph-msgs ros-jazzy-tf2-ros ros-jazzy-vision-msgs \
    ros-jazzy-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/simforge_venv
ENV PATH=/opt/simforge_venv/bin:$PATH
COPY src/simulation/sim_forge/requirements.txt /tmp/simforge_requirements.txt
RUN pip install --no-cache-dir -r /tmp/simforge_requirements.txt pytest==8.3.5
WORKDIR /opt/simforge_ws
COPY src/simulation/sim_forge src/sim_forge
RUN source /opt/ros/jazzy/setup.bash && python /usr/bin/colcon build --packages-select sim_forge
COPY watod_scripts/simforge/container_entrypoint.sh /usr/local/bin/simforge_entrypoint
ARG TASK_UID=1000
ARG TASK_GID=1000
RUN if ! getent group "${TASK_GID}" >/dev/null; then groupadd -g "${TASK_GID}" simforge; fi && \
    if ! getent passwd "${TASK_UID}" >/dev/null; then useradd -m -u "${TASK_UID}" -g "${TASK_GID}" simforge; fi && \
    mkdir -p /home/simforge && chown "${TASK_UID}:${TASK_GID}" /home/simforge && \
    chmod +x /usr/local/bin/simforge_entrypoint
ENV HOME=/home/simforge
ENV PATH=/home/simforge/.local/bin:/opt/simforge_venv/bin:$PATH
USER ${TASK_UID}:${TASK_GID}
WORKDIR /repo
ENTRYPOINT ["/usr/local/bin/simforge_entrypoint"]
CMD ["bash"]
