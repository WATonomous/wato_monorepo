#!/bin/bash

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
BAG_DIR="${1:?Usage: $0 <directory_mcap>}"

TOPICS=(
  /camera_pano_nn/image_rect_compressed
  /camera_pano_ne/image_rect_compressed
  /camera_pano_ee/image_rect_compressed
  /camera_pano_se/image_rect_compressed
  /camera_pano_ss/image_rect_compressed
  /camera_pano_sw/image_rect_compressed
  /camera_pano_ww/image_rect_compressed
  /camera_pano_nw/image_rect_compressed
  /camera_lower_ne/image_rect_compressed
  /camera_lower_se/image_rect_compressed
  /camera_lower_sw/image_rect_compressed
  /camera_lower_nw/image_rect_compressed
  /camera_pano_nn/camera_info
  /camera_pano_ne/camera_info
  /camera_pano_ee/camera_info
  /camera_pano_se/camera_info
  /camera_pano_ss/camera_info
  /camera_pano_sw/camera_info
  /camera_pano_ww/camera_info
  /camera_pano_nw/camera_info
  /camera_lower_ne/camera_info
  /camera_lower_se/camera_info
  /camera_lower_sw/camera_info
  /camera_lower_nw/camera_info
  /lidar_cc/velodyne_points
  /lidar_nw/velodyne_points
  /lidar_ne/velodyne_points
  /novatel/oem7/bestgnsspos
  /novatel/oem7/bestgnssvel
  /novatel/oem7/bestpos
  /novatel/oem7/bestutm
  /novatel/oem7/bestvel
  /novatel/oem7/corrimu
  /novatel/oem7/fix
  /novatel/oem7/gps
  /novatel/oem7/heading2
  /novatel/oem7/imu/data
  /novatel/oem7/imu/data_raw
  /novatel/oem7/insconfig
  /novatel/oem7/inspva
  /novatel/oem7/inspvax
  /novatel/oem7/insstdev
  /novatel/oem7/odom
  /novatel/oem7/odom_origin
  /novatel/oem7/oem7raw
  /novatel/oem7/ppppos
  /novatel/oem7/rxstatus
  /novatel/oem7/terrastarinfo
  /novatel/oem7/terrastarstatus
  /novatel/oem7/time
  /tf
  /tf_static
)

ros2 bag play "$BAG_DIR" --start-offset 9 --clock --topics "${TOPICS[@]}"
# ros2 bag play "$BAG_DIR" --clock --topics "${TOPICS[@]}"
