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
cmake_minimum_required(VERSION 3.8)
project(world_model_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Prediction.msg"
  "msg/WorldObject.msg"
  "msg/WorldObjectArray.msg"
  "msg/DynamicObject.msg"
  "msg/DynamicObjectArray.msg"
  "msg/AreaDefinition.msg"
  "msg/AreaOccupancyInfo.msg"
  "msg/AreaOccupancy.msg"
  "srv/GetDynamicObjectsByLanelet.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces vision_msgs
)

# Export dependency on generated interfaces for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
