find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${Eigen_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${Eigen_LIBRARIES})

link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
