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
install(DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install the generated URDF
install(FILES
  ${URDF_OUTPUT_FILE}
  DESTINATION share/${PROJECT_NAME}/urdf/
)

ament_package()
