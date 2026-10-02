FROM python:3.12-slim-bookworm
RUN pip install --no-cache-dir PyYAML==6.0.3
WORKDIR /app
COPY src/simulation/carla_ros_bridge/carla_common/carla_common /app/carla_common
COPY watod_scripts/tools/lanelet_to_xodr/prepare_track.py /app/prepare_track.py
ENTRYPOINT ["python", "/app/prepare_track.py"]
