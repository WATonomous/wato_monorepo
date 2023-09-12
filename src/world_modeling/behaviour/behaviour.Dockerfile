# ================= Dependencies ===================
# parent image is python since not using ROS atm, will change to humble once this is turned to a node
FROM python:3

RUN apt-get update && apt-get upgrade -y curl && \
    rm -rf /var/lib/apt/lists/*

# pip needs to install gymnasium and highway-env for the occupancy grid environment
RUN pip install gym
RUN pip install highway-env

COPY occupancy_mock.py occupancy_mock.py

CMD ["python", "occupancy_mock.py"]