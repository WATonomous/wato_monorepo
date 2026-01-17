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
project(carla_sample_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Generate URDF from xacro during build
set(XACRO_INPUT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/urdf/sample_vehicle.urdf.xacro)
set(URDF_OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/urdf/sample_vehicle.urdf)

# Get all xacro files in urdf directory
file(GLOB XACRO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro)

# Create output directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf)

# Run xacro to generate URDF
add_custom_command(
  OUTPUT ${URDF_OUTPUT_FILE}
  COMMAND xacro ${XACRO_INPUT_FILE} -o ${URDF_OUTPUT_FILE}
  DEPENDS ${XACRO_FILES}
  COMMENT "Generating URDF from xacro"
)

# Create a custom target that depends on the generated URDF
add_custom_target(generate_urdf ALL DEPENDS ${URDF_OUTPUT_FILE})

# Install directories
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the generated URDF
install(FILES
  ${URDF_OUTPUT_FILE}
  DESTINATION share/${PROJECT_NAME}/urdf/
)

ament_package()
