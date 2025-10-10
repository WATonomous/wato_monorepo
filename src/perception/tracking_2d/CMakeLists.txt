cmake_minimum_required(VERSION 3.10)
project(tracking_2d)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(tracking_2d_node src/tracking_2d_node.cpp)

target_include_directories(tracking_2d_node
    PUBLIC
    include
)

ament_target_dependencies(tracking_2d_node
    rclcpp
)

install(TARGETS
    tracking_2d_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()
