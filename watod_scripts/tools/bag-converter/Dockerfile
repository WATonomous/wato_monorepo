FROM python:3.12-slim

RUN pip install --no-cache-dir rosbags==0.11.0

WORKDIR /tool
COPY convert_ros2_mcap_to_ros1_bag.py /tool/convert_ros2_mcap_to_ros1_bag.py

ENTRYPOINT ["python3", "/tool/convert_ros2_mcap_to_ros1_bag.py"]
