ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/perception/perception:source_main

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for perception build
RUN git clone https://github.com/WATonomous/deep_ros.git deep_ros && \
    sed -i 's/find_package(Catch2 2 REQUIRED)/find_package(Catch2 REQUIRED)/' deep_ros/deep_test/cmake/add_deep_test.cmake && \
    find deep_ros -type f \( -name "*.cpp" -o -name "*.hpp" \) -exec sed -i 's|catch2/catch\.hpp|catch2/catch_all.hpp|g' {} + && \
    find deep_ros -type f \( -name "*.cpp" -o -name "*.hpp" \) -exec sed -i 's|cv_bridge/cv_bridge\.h|cv_bridge/cv_bridge.hpp|g' {} + && \
    find deep_ros -type f -name "*.cpp" -exec sed -i 's/\bApprox(/Catch::Approx(/g' {} + && \
    sed -i '/find_package(deep_msgs REQUIRED)/a find_package(class_loader REQUIRED)' deep_ros/deep_tools/camera_sync/CMakeLists.txt && \
    sed -i 's/target_link_libraries(multi_camera_sync_component/target_link_libraries(multi_camera_sync_component\n  class_loader::class_loader/' deep_ros/deep_tools/camera_sync/CMakeLists.txt
COPY src/perception perception
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies
