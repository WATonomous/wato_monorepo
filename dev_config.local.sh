#!/bin/bash
from dev_config.sh

# ACTIVE_PROFILES="data_stream"
# ACTIVE_PROFILES="data_stream lidar_publisher lidar_object_detection"
# ACTIVE_PROFILES="multimodal_object_detection"
# ACTIVE_PROFILES="data_stream sensor_fusion_publisher camera_detection"
ACTIVE_PROFILES="data_stream sensor_fusion_publisher lidar_object_detection"

FOXGLOVE_BRIDGE_PORT=8768