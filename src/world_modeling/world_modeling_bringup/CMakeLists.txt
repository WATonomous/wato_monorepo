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
project(world_modeling_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # Exactly one publisher (prediction_ml) on /world_modeling/world_object_seeds per bringup. The car
  # bringup also launches perception, whose file is checked when the source tree has it.
  set(SEEDS_CHECK ${CMAKE_CURRENT_SOURCE_DIR}/test/check_seeds_publishers.py)
  add_test(
    NAME seeds_single_publisher_car
    COMMAND python3 ${SEEDS_CHECK}
      ${CMAKE_CURRENT_SOURCE_DIR}/launch/world_modeling.launch.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/../../perception/perception_bringup/launch/perception.launch.yaml)
  add_test(
    NAME seeds_single_publisher_sim
    COMMAND python3 ${SEEDS_CHECK} ${CMAKE_CURRENT_SOURCE_DIR}/launch/world_modeling_sim.launch.yaml)
endif()

ament_package()
