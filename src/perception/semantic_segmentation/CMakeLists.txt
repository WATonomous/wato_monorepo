cmake_minimum_required(VERSION 3.8)
include(CMakePrintHelpers)
project(semantic_segmentation LANGUAGES CXX CUDA)
message(STATUS "Perception Utils Include Dirs: ${perception_utils_INCLUDE_DIRS}")
message(STATUS "Perception Utils Include Dirs: ${perception_utils_INCLUDE_DIRS}")

cmake_print_variables(PROJECT_SOURCE_DIR)
cmake_print_variables(perception_utils_INCLUDE_DIRS)
set(PADDLE_LIB "/paddle/paddle_inference_cuda120_build/paddle_inference_install_dir")

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Platform-independent settings
include_directories("${PADDLE_LIB}/")
set(PADDLE_LIB_THIRD_PARTY_PATH "${PADDLE_LIB}/third_party/install/")
include_directories(${PADDLE_LIB_THIRD_PARTY_PATH}protobuf/include
  ${PADDLE_LIB_THIRD_PARTY_PATH}glog/include
  ${PADDLE_LIB_THIRD_PARTY_PATH}gflags/include
  ${PADDLE_LIB_THIRD_PARTY_PATH}xxhash/include
  ${PADDLE_LIB_THIRD_PARTY_PATH}cryptopp/include
  ${PADDLE_LIB_THIRD_PARTY_PATH}onnxruntime/include
  ${PADDLE_LIB_THIRD_PARTY_PATH}paddle2onnx/include
  )

link_directories(${PADDLE_LIB_THIRD_PARTY_PATH}protobuf/lib
  ${PADDLE_LIB_THIRD_PARTY_PATH}glog/lib
  ${PADDLE_LIB_THIRD_PARTY_PATH}gflags/lib
  ${PADDLE_LIB_THIRD_PARTY_PATH}xxhash/lib
  ${PADDLE_LIB_THIRD_PARTY_PATH}cryptopp/lib
  ${PADDLE_LIB_THIRD_PARTY_PATH}onnxruntime/lib
  ${PADDLE_LIB_THIRD_PARTY_PATH}paddle2onnx/lib
  ${PADDLE_LIB}/paddle/lib)

  message(STATUS "Perception Utils Include Dirs: ${perception_utils_INCLUDE_DIRS}")

set(REQUIRED_PACKAGES
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  OpenCV
  cv_bridge
  CUDA
  ament_cmake
  yaml-cpp
  perception_utils
)

foreach(PACKAGE ${REQUIRED_PACKAGES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Include directories
include_directories(include 
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  /usr/local/include
  ${PADDLE_LIB}/paddle/include
  ${PADDLE_LIB}/third_party/install/mkldnn/include
  ${perception_utils_INCLUDE_DIRS}
)

# Link directories
link_directories(/usr/local/lib ${YAML_CPP_LIBRARIES})

# Set additional dependencies
set(EXTRA_LIBS "-lrt -ldl -lpthread")
set(DEPS ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${CUDA_LIBRARIES} ${YAML_CPP_LIBRARIES} ${EXTRA_LIBS}
${PADDLE_LIB}/paddle/lib/libpaddle_inference.so
${PADDLE_LIB}/paddle/lib/libphi.so
${PADDLE_LIB}/paddle/lib/libcommon.so
${PADDLE_LIB}/third_party/install/mkldnn/lib/libdnnl.so.3
${PADDLE_LIB}/third_party/install/mklml/lib/libiomp5.so
${PADDLE_LIB}/third_party/install/mklml/lib/libmklml_intel.so
)

# Add your executable
add_executable(semantic_segmentation src/semantic_segmentation_node.cpp)
target_link_libraries(semantic_segmentation ${DEPS})

# Additional installation commands
install(DIRECTORY launch config resource DESTINATION share/${PROJECT_NAME}/)
install(TARGETS semantic_segmentation DESTINATION lib/${PROJECT_NAME})
# Install shared libraries
install(FILES ${PADDLE_LIB}/paddle/lib/libpaddle_inference.so
              ${PADDLE_LIB}/paddle/lib/libphi.so
              ${PADDLE_LIB}/paddle/lib/libcommon.so
              ${PADDLE_LIB}/third_party/install/mkldnn/lib/libdnnl.so.3
              ${PADDLE_LIB}/third_party/install/mklml/lib/libiomp5.so
              ${PADDLE_LIB}/third_party/install/mklml/lib/libmklml_intel.so
        DESTINATION /usr/local/lib)

ament_target_dependencies(semantic_segmentation
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  perception_utils
)
ament_package()
