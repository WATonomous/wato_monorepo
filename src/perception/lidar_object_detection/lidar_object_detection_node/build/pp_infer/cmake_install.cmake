# Install script for directory: /home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/install/pp_infer")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pp_infer/pp_infer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pp_infer/pp_infer")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pp_infer/pp_infer"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pp_infer" TYPE EXECUTABLE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/pp_infer")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pp_infer/pp_infer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pp_infer/pp_infer")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pp_infer/pp_infer"
         OLD_RPATH "/usr/x86_64-linux-gnu/lib:/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build:/opt/ros/humble/install/vision_msgs/lib:/opt/ros/humble/install/pcl_ros/lib:/opt/ros/humble/install/visualization_msgs/lib:/opt/ros/humble/install/message_filters/lib:/opt/ros/humble/install/builtin_interfaces/lib:/opt/ros/humble/install/geometry_msgs/lib:/opt/ros/humble/install/rosidl_typesupport_fastrtps_c/lib:/opt/ros/humble/install/rmw/lib:/opt/ros/humble/install/rosidl_typesupport_fastrtps_cpp/lib:/opt/ros/humble/install/rosidl_typesupport_cpp/lib:/opt/ros/humble/install/rosidl_typesupport_introspection_cpp/lib:/opt/ros/humble/install/rcpputils/lib:/opt/ros/humble/install/rosidl_typesupport_c/lib:/opt/ros/humble/install/rosidl_typesupport_introspection_c/lib:/opt/ros/humble/install/rcutils/lib:/opt/ros/humble/install/rosidl_runtime_c/lib:/opt/ros/humble/install/pcl_msgs/lib:/opt/ros/humble/install/rclcpp/lib:/opt/ros/humble/install/sensor_msgs/lib:/opt/ros/humble/install/std_msgs/lib:/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer:/opt/ros/humble/install/rclcpp_components/lib:/opt/ros/humble/install/class_loader/lib:/opt/ros/humble/install/composition_interfaces/lib:/opt/ros/humble/install/orocos_kdl_vendor/lib:/opt/ros/humble/install/tf2_ros/lib:/opt/ros/humble/install/tf2/lib:/opt/ros/humble/install/console_bridge_vendor/lib:/opt/ros/humble/install/rclcpp_action/lib:/opt/ros/humble/install/libstatistics_collector/lib:/opt/ros/humble/install/rosgraph_msgs/lib:/opt/ros/humble/install/statistics_msgs/lib:/opt/ros/humble/install/rcl_action/lib:/opt/ros/humble/install/rcl/lib:/opt/ros/humble/install/rcl_interfaces/lib:/opt/ros/humble/install/rcl_yaml_param_parser/lib:/opt/ros/humble/install/tracetools/lib:/opt/ros/humble/install/rmw_implementation/lib:/opt/ros/humble/install/ament_index_cpp/lib:/opt/ros/humble/install/rcl_logging_spdlog/lib:/opt/ros/humble/install/rcl_logging_interface/lib:/opt/ros/humble/install/tf2_msgs/lib:/opt/ros/humble/install/action_msgs/lib:/opt/ros/humble/install/unique_identifier_msgs/lib:/opt/ros/humble/install/libyaml_vendor/lib:/opt/ros/humble/install/fastcdr/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pp_infer/pp_infer")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer/" TYPE DIRECTORY FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/pp_infer")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/pp_infer")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer/environment" TYPE FILE FILES "/opt/ros/humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer/environment" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer/environment" TYPE FILE FILES "/opt/ros/humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer/environment" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_index/share/ament_index/resource_index/packages/pp_infer")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer/cmake" TYPE FILE FILES
    "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_core/pp_inferConfig.cmake"
    "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/ament_cmake_core/pp_inferConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pp_infer" TYPE FILE FILES "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/build/pp_infer/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
