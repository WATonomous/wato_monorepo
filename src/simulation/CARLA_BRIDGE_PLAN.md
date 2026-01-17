# CARLA ROS2 Bridge Implementation Plan

## Overview
Create a multi-package ROS2 workspace for interfacing with CARLA C++ Client API. The architecture is organized by layer with clear separation of concerns. All bridge nodes are lifecycle nodes AND composable components. A scenario server dynamically loads C++ scenario plugins and manages lifecycle transitions of all bridge nodes.

## Multi-Package Architecture

```
/home/bolty/ament_ws/src/simulation/carla_ros_bridge/      # Meta-package directory
│
├── carla_ros_bridge/                           # Meta-package (depends on all)
│   ├── package.xml
│   └── CMakeLists.txt
│
├── carla_msgs/                                 # Message and service definitions
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── msg/
│   │   ├── AckermannControl.msg               # Steering angle + linear acceleration <--- should use AckermannDriveStamed https://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
│   │   └── ScenarioStatus.msg                 # Current scenario status
│   └── srv/
│       ├── SwitchScenario.srv                 # Switch scenario plugin
│       └── GetAvailableScenarios.srv          # List available scenarios
│
├── carla_core/                                 # Core CARLA client wrapper
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/carla_core/
│   │   ├── carla_client.hpp                   # CARLA C++ API wrapper
│   │   ├── sensor_factory.hpp                 # Sensor creation/management
│   │   └── visibility_control.hpp             # Export macros
│   └── src/
│       ├── carla_client.cpp
│       └── sensor_factory.cpp
│
├── carla_scenarios/                            # Scenario plugin system <--- i want to give external packages the ability to add and register their own plugin scenarios
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/carla_scenarios/
│   │   ├── scenario_base.hpp                  # Base class for plugins
│   │   ├── scenario_server_node.hpp           # Scenario server (lifecycle + component)
│   │   └── visibility_control.hpp
│   ├── src/
│   │   └── scenario_server_node.cpp
│   ├── plugins/
│   │   ├── highway_scenario.cpp               # Example scenario
│   │   ├── urban_scenario.cpp
│   │   └── parking_scenario.cpp
│   └── plugins.xml                            # Pluginlib description
│
├── carla_control/                              # Vehicle control bridge
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/carla_control/
│   │   ├── ackermann_control_node.hpp         # Ackermann control (lifecycle + component)
│   │   ├── pid_controller.hpp                 # PID for accel → throttle/brake
│   │   └── visibility_control.hpp
│   └── src/
│       ├── ackermann_control_node.cpp
│       └── pid_controller.cpp
│
└── carla_perception/                           # Sensor bridges
    ├── package.xml
    ├── CMakeLists.txt
    ├── include/carla_perception/
    │   ├── camera_publisher_node.hpp          # RGB camera (lifecycle + component)
    │   ├── lidar_publisher_node.hpp           # LiDAR (lifecycle + component)
    │   ├── bbox_publisher_node.hpp            # 3D BBoxes (lifecycle + component)
    │   ├── sensor_config_parser.hpp           # URDF parser for sensor placement
    │   └── visibility_control.hpp
    └── src/
        ├── camera_publisher_node.cpp
        ├── lidar_publisher_node.cpp
        ├── bbox_publisher_node.cpp
        └── sensor_config_parser.cpp
```

## Key Architecture Decisions

### 1. Multi-Package Organization by Layer
- **carla_msgs**: Message/service definitions (no dependencies)
- **carla_core**: CARLA C++ client wrapper, sensor factory (depends on: carla_msgs)
- **carla_scenarios**: Scenario plugins and server node (depends on: carla_core, carla_msgs)
- **carla_control**: Vehicle control node (depends on: carla_core, carla_msgs)
- **carla_perception**: Sensor publisher nodes (depends on: carla_core, carla_msgs)
- **carla_ros_bridge**: Meta-package (depends on all above)

### 2. Lifecycle Nodes + Composable Components
All bridge nodes are:
- **Lifecycle nodes** (inherit from `rclcpp_lifecycle::LifecycleNode`)
- **Composable components** (registered with `RCLCPP_COMPONENTS_REGISTER_NODE`)

Benefits:
- Flexible deployment (same process or separate processes)
- State management via lifecycle state machine
- Scenario switching without restarting nodes

### 3. Bridge Nodes (One Per Function)
- `scenario_server` (carla_scenarios) - Manages scenarios and coordinates other nodes
- `ackermann_control` (carla_control) - Ackermann control: steering angle + linear acceleration
- `camera_publisher` (carla_perception) - RGB images + camera info
- `lidar_publisher` (carla_perception) - LiDAR point clouds
- `bbox_publisher` (carla_perception) - 3D bounding boxes <--- and maybe 2d

### 4. Ackermann Control Interface
- Input: `carla_msgs/AckermannControl` with:
  - `steering_angle` (radians): Target steering angle
  - `linear_acceleration` (m/s²): Desired acceleration
- Conversion: PID controller tracks acceleration by adjusting throttle/brake
- Feedback: Uses current velocity from CARLA to compute control commands

### 5. Sensor Placement from robot_description
- Nodes read `robot_description` parameter (URDF/xacro)
- Parse URDF to find sensor links (e.g., `camera_link`, `lidar_link`)
- Extract transform from base_link to sensor link
- Spawn CARLA sensors at corresponding locations
- Publish with correct TF frames

### 6. Scenario Plugin System
Scenarios are C++ plugins loaded via pluginlib:
- Base class: `carla_scenarios::ScenarioBase`
- Plugin interface: initialize(), setup(), execute(), cleanup()
- Service to switch scenarios at runtime
- When switching, scenario_server triggers lifecycle transitions on all bridge nodes

### 7. Lifecycle State Flow for Scenario Switching

```
1. Service call: /switch_scenario
2. Scenario Server deactivates all managed nodes
   - ackermann_control: Active → Inactive
   - camera_publisher: Active → Inactive
   - lidar_publisher: Active → Inactive
   - bbox_publisher: Active → Inactive
3. Scenario Server unloads current scenario plugin
4. Scenario Server loads new scenario plugin
5. New scenario initializes and sets up CARLA world
6. Scenario Server activates all managed nodes
   - Nodes reset internal state on activation
   - Nodes reconnect to CARLA and obtain current simulation state
   - Sensors are respawned based on robot_description
7. New scenario starts executing
```

### 8. Modern CMake (No ament_target_dependencies)
Use target-based linking instead of deprecated ament_target_dependencies:

```cmake
# Old (deprecated)
ament_target_dependencies(my_target rclcpp std_msgs)

# New (modern)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
target_link_libraries(my_target
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
```

## Critical Files to Implement

The implementation follows dependency order: carla_msgs → carla_core → carla_scenarios/carla_control/carla_perception → carla_ros_bridge (meta-package).

### Package 1: carla_msgs

**1.1. carla_msgs/package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>carla_msgs</name>
  <version>1.0.0</version>
  <description>Message and service definitions for CARLA ROS2 bridge</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**1.2. carla_msgs/CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.10)
project(carla_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AckermannControl.msg"
  "msg/ScenarioStatus.msg"
)

set(srv_files
  "srv/SwitchScenario.srv"
  "srv/GetAvailableScenarios.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
```

**1.3. carla_msgs/msg/AckermannControl.msg**

```
std_msgs/Header header

# Target steering angle in radians
# Positive = left, Negative = right
float64 steering_angle

# Desired linear acceleration in m/s²
# Positive = accelerate, Negative = decelerate
float64 linear_acceleration
```

**1.4. carla_msgs/msg/ScenarioStatus.msg**

```
std_msgs/Header header

# Name of currently active scenario
string scenario_name

# Human-readable description
string description

# Scenario state: loading, running, completed, error
string state

# Additional information (e.g., error messages)
string info
```

**1.5. carla_msgs/srv/SwitchScenario.srv**

```
# Plugin class name, e.g., "carla_scenarios/HighwayScenario"
string scenario_name
---
bool success
string message
string previous_scenario
```

**1.6. carla_msgs/srv/GetAvailableScenarios.srv**

```
# Request - empty
---
# List of available scenario plugin class names
string[] scenario_names

# Human-readable descriptions
string[] descriptions
```

### Package 2: carla_core

**2.1. carla_core/package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>carla_core</name>
  <version>1.0.0</version>
  <description>Core CARLA C++ client wrapper and utilities</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>carla_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**2.2. carla_core/CMakeLists.txt** (Modern CMake, no ament_target_dependencies)

```cmake
cmake_minimum_required(VERSION 3.10)
project(carla_core)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find CARLA C++ Client Library
set(CARLA_INCLUDE_DIR "/usr/local/include/carla" CACHE PATH "CARLA include directory")
set(CARLA_LIB_DIR "/usr/local/lib" CACHE PATH "CARLA library directory")

find_library(CARLA_CLIENT_LIB carla_client PATHS ${CARLA_LIB_DIR} REQUIRED)
find_library(RPC_LIB rpc PATHS ${CARLA_LIB_DIR})

include_directories(include ${CARLA_INCLUDE_DIR})

# CARLA Client library
add_library(carla_client SHARED
  src/carla_client.cpp
  src/sensor_factory.cpp
)

target_link_libraries(carla_client
  rclcpp::rclcpp
  ${carla_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${CARLA_CLIENT_LIB}
  ${RPC_LIB}
)

install(TARGETS carla_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(carla_client)
ament_export_dependencies(rclcpp carla_msgs geometry_msgs sensor_msgs)

ament_package()
```

**2.3. carla_core/include/carla_core/carla_client.hpp**
- Class `CarlaClient`: Wrapper around CARLA C++ API
- Methods: connect(), disconnect(), getWorld(), getEgoVehicle(), getAllActors(), isConnected()
- Connection management, reconnection logic, error handling

**2.4. carla_core/include/carla_core/sensor_factory.hpp**
- Class `SensorFactory`: Creates and manages CARLA sensors
- Methods: createCamera(), createLidar(), attachSensorToVehicle()
- Sensor configuration from URDF transforms

**2.5. carla_core/include/carla_core/visibility_control.hpp**
- Export macros for shared library symbols
- Required for component exports

### Package 3: carla_scenarios

**3.1. carla_scenarios/package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>carla_scenarios</name>
  <version>1.0.0</version>
  <description>Scenario plugin system and scenario server node</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>rclcpp_components</depend>
  <depend>lifecycle_msgs</depend>
  <depend>pluginlib</depend>
  <depend>carla_core</depend>
  <depend>carla_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <pluginlib plugin="${prefix}/plugins.xml"/>
  </export>
</package>
```

**3.2. carla_scenarios/CMakeLists.txt** (Modern CMake)

```cmake
cmake_minimum_required(VERSION 3.10)
project(carla_scenarios)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(carla_core REQUIRED)
find_package(carla_msgs REQUIRED)

include_directories(include)

# Scenario Server Node (lifecycle + component)
add_library(scenario_server_node SHARED
  src/scenario_server_node.cpp
)

target_link_libraries(scenario_server_node
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${lifecycle_msgs_TARGETS}
  pluginlib::pluginlib
  carla_core::carla_client
  ${carla_msgs_TARGETS}
)

rclcpp_components_register_nodes(scenario_server_node
  "carla_scenarios::ScenarioServerNode")

# Scenario Plugins Library
add_library(carla_scenarios_plugins SHARED
  plugins/highway_scenario.cpp
  plugins/urban_scenario.cpp
  plugins/parking_scenario.cpp
)

target_link_libraries(carla_scenarios_plugins
  rclcpp::rclcpp
  pluginlib::pluginlib
  carla_core::carla_client
)

pluginlib_export_plugin_description_file(carla_scenarios plugins.xml)

install(TARGETS scenario_server_node carla_scenarios_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(scenario_server_node carla_scenarios_plugins)
ament_export_dependencies(rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs pluginlib carla_core carla_msgs)

ament_package()
```

**3.3. carla_scenarios/include/carla_scenarios/scenario_base.hpp**
- Pure virtual base class for scenario plugins
- Methods: initialize(), setup(), execute(), cleanup(), getName(), getDescription()

**3.4. carla_scenarios/include/carla_scenarios/scenario_server_node.hpp**
- Inherits `rclcpp_lifecycle::LifecycleNode`
- Lifecycle callbacks
- Pluginlib loader for scenarios
- Services: /switch_scenario, /get_available_scenarios
- Service clients for managing lifecycle of other nodes

**3.5. carla_scenarios/plugins/highway_scenario.cpp**
- Example scenario implementation
- `PLUGINLIB_EXPORT_CLASS(carla_scenarios::HighwayScenario, carla_scenarios::ScenarioBase)`

**3.6. carla_scenarios/plugins.xml**
- Pluginlib XML description of scenario plugins

### Package 4: carla_control

**4.1. carla_control/package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>carla_control</name>
  <version>1.0.0</version>
  <description>Vehicle control bridge node</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>rclcpp_components</depend>
  <depend>lifecycle_msgs</depend>
  <depend>carla_core</depend>
  <depend>carla_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**4.2. carla_control/CMakeLists.txt** (Modern CMake)

```cmake
cmake_minimum_required(VERSION 3.10)
project(carla_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(carla_core REQUIRED)
find_package(carla_msgs REQUIRED)

include_directories(include)

# PID Controller library
add_library(pid_controller SHARED
  src/pid_controller.cpp
)

# Ackermann Control Node (lifecycle + component)
add_library(ackermann_control_node SHARED
  src/ackermann_control_node.cpp
)

target_link_libraries(ackermann_control_node
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${lifecycle_msgs_TARGETS}
  carla_core::carla_client
  ${carla_msgs_TARGETS}
  pid_controller
)

rclcpp_components_register_nodes(ackermann_control_node
  "carla_control::AckermannControlNode")

install(TARGETS pid_controller ackermann_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(pid_controller ackermann_control_node)
ament_export_dependencies(rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs carla_core carla_msgs)

ament_package()
```

**4.3. carla_control/include/carla_control/ackermann_control_node.hpp**
- Inherits `rclcpp_lifecycle::LifecycleNode`
- Lifecycle callbacks
- Subscription: ~/command → carla_msgs/AckermannControl
- PID controller to convert acceleration → throttle/brake
- Applies control to CARLA ego vehicle

**4.4. carla_control/include/carla_control/pid_controller.hpp**
- PID controller implementation
- Methods: compute(), reset(), setGains()
- Tracks desired acceleration by adjusting throttle/brake

### Package 5: carla_perception

**5.1. carla_perception/package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>carla_perception</name>
  <version>1.0.0</version>
  <description>Sensor bridge nodes (camera, lidar, bbox)</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>rclcpp_components</depend>
  <depend>lifecycle_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>urdf</depend>
  <depend>carla_core</depend>
  <depend>carla_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**5.2. carla_perception/CMakeLists.txt** (Modern CMake)

```cmake
cmake_minimum_required(VERSION 3.10)
project(carla_perception)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(carla_core REQUIRED)
find_package(carla_msgs REQUIRED)

include_directories(include)

# Sensor Config Parser library
add_library(sensor_config_parser SHARED
  src/sensor_config_parser.cpp
)

target_link_libraries(sensor_config_parser
  urdf::urdf
)

# Camera Publisher Node (lifecycle + component)
add_library(camera_publisher_node SHARED
  src/camera_publisher_node.cpp
)

target_link_libraries(camera_publisher_node
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${lifecycle_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  carla_core::carla_client
  sensor_config_parser
)

rclcpp_components_register_nodes(camera_publisher_node
  "carla_perception::CameraPublisherNode")

# LiDAR Publisher Node (lifecycle + component)
add_library(lidar_publisher_node SHARED
  src/lidar_publisher_node.cpp
)

target_link_libraries(lidar_publisher_node
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${lifecycle_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  carla_core::carla_client
  sensor_config_parser
)

rclcpp_components_register_nodes(lidar_publisher_node
  "carla_perception::LidarPublisherNode")

# BBox Publisher Node (lifecycle + component)
add_library(bbox_publisher_node SHARED
  src/bbox_publisher_node.cpp
)

target_link_libraries(bbox_publisher_node
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${lifecycle_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  carla_core::carla_client
)

rclcpp_components_register_nodes(bbox_publisher_node
  "carla_perception::BBoxPublisherNode")

install(TARGETS
  sensor_config_parser
  camera_publisher_node
  lidar_publisher_node
  bbox_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(sensor_config_parser camera_publisher_node lidar_publisher_node bbox_publisher_node)
ament_export_dependencies(rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs sensor_msgs vision_msgs urdf carla_core carla_msgs)

ament_package()
```

**5.3. carla_perception/include/carla_perception/sensor_config_parser.hpp**
- Class `SensorConfigParser`: Parses robot_description URDF
- Methods: findSensorLinks(), getSensorTransform()
- Extracts sensor link names and transforms relative to base_link

**5.4. carla_perception/include/carla_perception/camera_publisher_node.hpp**
- Inherits `rclcpp_lifecycle::LifecycleNode`
- Lifecycle callbacks
- Publishers:
  - ~/image_raw → sensor_msgs/Image (RGB)
  - ~/camera_info → sensor_msgs/CameraInfo
- Spawns CARLA camera based on robot_description

**5.5. carla_perception/include/carla_perception/lidar_publisher_node.hpp**
- Inherits `rclcpp_lifecycle::LifecycleNode`
- Lifecycle callbacks
- Publisher: ~/points → sensor_msgs/PointCloud2
- Spawns CARLA LiDAR based on robot_description

**5.6. carla_perception/include/carla_perception/bbox_publisher_node.hpp**
- Inherits `rclcpp_lifecycle::LifecycleNode`
- Lifecycle callbacks
- Publisher: ~/detections → vision_msgs/Detection3DArray
- Queries CARLA for all actor bounding boxes

### Package 6: carla_ros_bridge (Meta-package)

**6.1. carla_ros_bridge/package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>carla_ros_bridge</name>
  <version>1.0.0</version>
  <description>Meta-package for CARLA ROS2 bridge</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>carla_msgs</exec_depend>
  <exec_depend>carla_core</exec_depend>
  <exec_depend>carla_scenarios</exec_depend>
  <exec_depend>carla_control</exec_depend>
  <exec_depend>carla_perception</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**6.2. carla_ros_bridge/CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.10)
project(carla_ros_bridge)

find_package(ament_cmake REQUIRED)

# Meta-package has no build targets
# Just depends on all other packages

ament_package()
```

## Implementation Sequence

Follow this order to respect package dependencies:

### Phase 1: carla_msgs (No dependencies)
1. Create package directory structure
2. Write package.xml and CMakeLists.txt
3. Define messages: AckermannControl.msg, ScenarioStatus.msg
4. Define services: SwitchScenario.srv, GetAvailableScenarios.srv
5. Build and verify message generation

### Phase 2: carla_core (Depends on: carla_msgs)
1. Create package directory structure
2. Write package.xml and CMakeLists.txt (modern CMake)
3. Implement visibility_control.hpp
4. Implement carla_client.hpp/cpp:
   - CARLA C++ API wrapper
   - Connection management (connect, disconnect, reconnect)
   - World/vehicle/actor access
   - Error handling
5. Implement sensor_factory.hpp/cpp:
   - Sensor creation and configuration
   - Sensor attachment to vehicles
   - Transform handling
6. Build and verify library exports

### Phase 3: carla_scenarios (Depends on: carla_core, carla_msgs)
1. Create package directory structure
2. Write package.xml and CMakeLists.txt
3. Implement scenario_base.hpp:
   - Pure virtual base class
   - Plugin interface methods
4. Implement scenario_server_node.hpp/cpp:
   - Lifecycle node + component
   - Pluginlib loader
   - Service handlers for switching scenarios
   - Lifecycle clients for managing other nodes
5. Implement example scenario plugins:
   - highway_scenario.cpp
   - urban_scenario.cpp
   - parking_scenario.cpp
6. Write plugins.xml
7. Register components
8. Build and verify component registration

### Phase 4: carla_control (Depends on: carla_core, carla_msgs)
1. Create package directory structure
2. Write package.xml and CMakeLists.txt
3. Implement pid_controller.hpp/cpp:
   - PID controller for acceleration tracking
   - Methods: compute(), reset(), setGains()
4. Implement ackermann_control_node.hpp/cpp:
   - Lifecycle node + component
   - Subscribe to AckermannControl messages
   - PID-based conversion: acceleration → throttle/brake
   - Apply control to CARLA ego vehicle
   - Timeout safety
5. Register component
6. Build and verify component registration

### Phase 5: carla_perception (Depends on: carla_core, carla_msgs)
1. Create package directory structure
2. Write package.xml and CMakeLists.txt
3. Implement sensor_config_parser.hpp/cpp:
   - Parse robot_description URDF
   - Extract sensor links and transforms
4. Implement camera_publisher_node.hpp/cpp:
   - Lifecycle node + component
   - Read robot_description for camera placement
   - Spawn CARLA camera sensor
   - Publish sensor_msgs/Image (RGB)
   - Publish sensor_msgs/CameraInfo
5. Implement lidar_publisher_node.hpp/cpp:
   - Lifecycle node + component
   - Read robot_description for LiDAR placement
   - Spawn CARLA LiDAR sensor
   - Publish sensor_msgs/PointCloud2
6. Implement bbox_publisher_node.hpp/cpp:
   - Lifecycle node + component
   - Query CARLA for all actors
   - Convert bounding boxes to vision_msgs/Detection3DArray
7. Register all components
8. Build and verify component registration

### Phase 6: carla_ros_bridge (Meta-package)
1. Create meta-package directory
2. Write package.xml (depends on all other packages)
3. Write minimal CMakeLists.txt
4. Build entire workspace

### Phase 7: Launch Files and Configuration
1. Create launch files in simulation_bringup or carla_scenarios:
   - Component container launch (all nodes in one process)
   - Separate process launch (debugging)
2. Create configuration files:
   - CARLA connection parameters
   - Vehicle control PID gains
   - Sensor parameters
   - Initial scenario selection
3. Create example robot_description URDF with sensor links

### Phase 8: Testing and Documentation
1. Test message/service definitions
2. Test CARLA connection and reconnection
3. Test lifecycle transitions for each node
4. Test scenario switching
5. Test sensor data publication
6. Test Ackermann control
7. Write comprehensive README with:
   - Installation instructions
   - CARLA C++ client setup
   - Usage examples
   - Troubleshooting

## Component Registration Pattern

Each node library must register its component in the .cpp file:

```cpp
#include <rclcpp_components/register_node_macro.hpp>

// At the end of the implementation file
RCLCPP_COMPONENTS_REGISTER_NODE(carla_scenarios::ScenarioServerNode)
```

And in CMakeLists.txt:

```cmake
rclcpp_components_register_nodes(scenario_server_node
  "carla_scenarios::ScenarioServerNode")
```

## Dependencies to Install

Before building, ensure these are installed: <--- all dependencies MUST be managed through rosdep package.xml

```bash
# ROS2 Jazzy packages
sudo apt install ros-jazzy-vision-msgs
sudo apt install ros-jazzy-rclcpp-components
sudo apt install ros-jazzy-urdf

# CARLA C++ Client Library (0.10.0)
# Must be installed from CARLA distribution or built from source
# Typical installation:
#   - Headers in /usr/local/include/carla
#   - Libraries (libcarla_client.so, librpc.so) in /usr/local/lib
# Adjust CARLA_INCLUDE_DIR and CARLA_LIB_DIR in CMakeLists.txt if different
```

## Verification Steps

After implementation, verify each package in dependency order:

### 1. Build carla_msgs

```bash
cd /home/bolty/ament_ws
colcon build --packages-select carla_msgs
source install/setup.bash

# Verify messages
ros2 interface show carla_msgs/msg/AckermannControl
ros2 interface show carla_msgs/srv/SwitchScenario
```

### 2. Build carla_core

```bash
colcon build --packages-select carla_core
source install/setup.bash

# Verify library exports
ls install/carla_core/lib/libcarla_client.so
```

### 3. Build carla_scenarios

```bash
colcon build --packages-select carla_scenarios
source install/setup.bash

# Verify component registration
ros2 component types | grep carla_scenarios
# Should show: carla_scenarios::ScenarioServerNode

# Verify plugin registration
ros2 pkg plugins --package carla_scenarios carla_scenarios::ScenarioBase
# Should list: HighwayScenario, UrbanScenario, ParkingScenario
```

### 4. Build carla_control

```bash
colcon build --packages-select carla_control
source install/setup.bash

# Verify component registration
ros2 component types | grep carla_control
# Should show: carla_control::AckermannControlNode
```

### 5. Build carla_perception

```bash
colcon build --packages-select carla_perception
source install/setup.bash

# Verify component registration
ros2 component types | grep carla_perception
# Should show: CameraPublisherNode, LidarPublisherNode, BBoxPublisherNode
```

### 6. Build entire workspace

```bash
colcon build
source install/setup.bash
```

### 7. Runtime Testing

**7.1. Start CARLA simulator** <--- carla is up and running in a separate container at the moment

```bash
# Assuming CARLA is installed
cd /path/to/CARLA
./CarlaUE4.sh
```

**7.2. Launch bridge nodes**

```bash
# Launch all nodes (add launch file to simulation_bringup or carla_scenarios)
ros2 launch carla_scenarios carla_bridge.launch.py
```

**7.3. Verify nodes and lifecycle states**

```bash
# List nodes
ros2 node list
# Expected: /scenario_server, /ackermann_control, /camera_publisher, /lidar_publisher, /bbox_publisher

# Check lifecycle states
ros2 lifecycle nodes
ros2 lifecycle get /ackermann_control
# Expected state: active [3]
```

**7.4. Test scenario switching** <--- when a scenario switch occurs, the other nodes should be forced to restart (lifecycle wise)

```bash
# Get available scenarios
ros2 service call /scenario_server/get_available_scenarios carla_msgs/srv/GetAvailableScenarios

# Switch to urban scenario
ros2 service call /scenario_server/switch_scenario carla_msgs/srv/SwitchScenario "{scenario_name: 'carla_scenarios/UrbanScenario'}"
```

**7.5. Test Ackermann control**

```bash
# Send control command
ros2 topic pub --once /ackermann_control/command carla_msgs/msg/AckermannControl \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, steering_angle: 0.1, linear_acceleration: 2.0}"

# Verify vehicle moves in CARLA
```

**7.6. Verify sensor topics**

```bash
# List topics
ros2 topic list
# Expected: /camera_publisher/image_raw, /camera_publisher/camera_info,
#           /lidar_publisher/points, /bbox_publisher/detections

# Echo camera images
ros2 topic hz /camera_publisher/image_raw

# Echo LiDAR points
ros2 topic hz /lidar_publisher/points

# Echo bounding boxes
ros2 topic echo /bbox_publisher/detections --once
```

**7.7. Verify robot_description integration**

```bash
# Set robot_description parameter (example URDF with sensor links)
ros2 param set /camera_publisher robot_description "$(cat example_robot.urdf)"

# Restart node to respawn sensors at correct positions
ros2 lifecycle set /camera_publisher deactivate
ros2 lifecycle set /camera_publisher activate
```

## Key Technical Notes

### Modern CMake Linking
Use modern CMake targets instead of ament_target_dependencies:
- ROS2 packages: `rclcpp::rclcpp`, `${std_msgs_TARGETS}`, `${sensor_msgs_TARGETS}`
- CARLA: `${CARLA_CLIENT_LIB}`, `${RPC_LIB}`
- Custom libraries: `carla_core::carla_client`, `pluginlib::pluginlib`, `urdf::urdf`

### URDF Sensor Placement <-- can you make note of what a "standard" description package looks like?
- Sensor publisher nodes read `robot_description` parameter
- Parse URDF to find links with sensor names (e.g., `camera_link`, `lidar_link`)
- Extract transform from `base_link` to sensor link
- Spawn CARLA sensors at corresponding offsets from ego vehicle
- Publish with correct frame_id matching URDF link name

### Lifecycle State Flow
- **Unconfigured → Configure**: Load parameters, connect to CARLA
- **Inactive → Activate**: Create publishers/subscriptions, spawn sensors, start timers
- **Active → Deactivate**: Stop timers, destroy sensors, reset state
- **Inactive → Cleanup**: Disconnect from CARLA, release resources

### PID Controller for Ackermann
- Input: Desired linear acceleration (m/s²)
- Feedback: Current velocity from CARLA ego vehicle
- Output: Throttle [0-1] or Brake [0-1]
- Control loop: `error = desired_accel - (current_velocity - prev_velocity) / dt`
- PID computes throttle/brake to minimize error

### Error Handling
- CARLA connection failures: Attempt reconnection with exponential backoff
- Sensor spawn failures: Log error, continue with other sensors
- Lifecycle error state: Trigger cleanup and allow manual recovery

## End Goal

A production-ready multi-package ROS2 system:
- ✅ Multi-package architecture with clear separation of concerns
- ✅ All nodes are lifecycle nodes AND composable components
- ✅ Scenario server with pluginlib for dynamic scenario loading
- ✅ Scenario switching triggers coordinated lifecycle transitions
- ✅ Ackermann control with PID-based acceleration tracking
- ✅ Camera publisher (RGB + camera info)
- ✅ LiDAR publisher (PointCloud2)
- ✅ 3D bounding box publisher (Detection3DArray)
- ✅ Sensor placement from robot_description URDF
- ✅ Modern CMake (no ament_target_dependencies)
- ✅ Flexible deployment (component container or separate processes)
- ✅ Clean separation: each bridge is its own node in its own package
