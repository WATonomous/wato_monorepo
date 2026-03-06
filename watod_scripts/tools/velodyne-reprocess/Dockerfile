# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-jazzy-velodyne-msgs \
    ros-jazzy-velodyne-pointcloud \
    ros-jazzy-rclcpp-components \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    procps \
  && rm -rf /var/lib/apt/lists/*

COPY run_packets_to_points_mcap.sh /tool/run_packets_to_points_mcap.sh
RUN chmod +x /tool/run_packets_to_points_mcap.sh

ENTRYPOINT ["/tool/run_packets_to_points_mcap.sh"]
