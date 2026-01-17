# Implementation Notes - User Feedback Addressed

## Changes from Original Plan

### 1. Use ackermann_msgs/AckermannDriveStamped ✅
- **Original**: Custom `carla_msgs/AckermannControl.msg`
- **Updated**: Use standard `ackermann_msgs/AckermannDriveStamped`
- **Reason**: Standard ROS2 message, better interoperability
- **Impact**: Remove AckermannControl.msg, add `ackermann_msgs` dependency to carla_control

### 2. External Scenario Plugin Support ✅
- **Requirement**: External packages should be able to register their own scenario plugins
- **Implementation**:
  - `scenario_base.hpp` is a pure virtual base class exported by carla_scenarios
  - External packages can:
    1. Depend on carla_scenarios
    2. Include `<carla_scenarios/scenario_base.hpp>`
    3. Implement their own scenario class
    4. Create their own `plugins.xml` with `<pluginlib plugin="${prefix}/plugins.xml"/>`
    5. Export the plugin via `pluginlib_export_plugin_description_file(carla_scenarios plugins.xml)`
  - ScenarioServerNode uses pluginlib::ClassLoader which automatically discovers all registered plugins

### 3. Add 2D Bounding Boxes ✅
- **Original**: Only 3D bounding boxes (vision_msgs/Detection3DArray)
- **Updated**: Support both 2D and 3D bounding boxes
- **Implementation**:
  - bbox_publisher_node will publish both:
    - `~/detections_3d` → vision_msgs/Detection3DArray
    - `~/detections_2d` → vision_msgs/Detection2DArray
  - CARLA provides 3D bounding boxes, project to 2D using camera intrinsics

### 4. Dependencies via rosdep Only ✅
- **Requirement**: All dependencies must be managed through package.xml (rosdep)
- **Implementation**:
  - All ROS2 packages declared in package.xml using rosdep keys:
    - `vision-msgs` → ros-jazzy-vision-msgs
    - `ackermann-msgs` → ros-jazzy-ackermann-msgs
    - `rclcpp-components` → ros-jazzy-rclcpp-components
    - `urdf` → ros-jazzy-urdf
  - CARLA C++ Client Library:
    - Cannot be installed via rosdep (not in ROS repos)
    - Will be found via CMake find_library()
    - Users must install manually or provide path via CMAKE_PREFIX_PATH
    - Document installation in README

### 5. CARLA in Container ✅
- **Status**: CARLA is already running in a separate container
- **Implementation**:
  - Connect via TCP to CARLA_HOSTNAME:2000 (configurable)
  - No need to launch CARLA from ROS2
  - Connection parameters in launch files

### 6. Lifecycle Restart on Scenario Switch ✅
- **Requirement**: When scenario switches, other nodes must restart (lifecycle transitions)
- **Implementation**: Already in design
  - ScenarioServerNode calls lifecycle service `/node_name/change_state`
  - Sequence: Active → Deactivate → Cleanup → Configure → Activate
  - This forces nodes to reset all state and reconnect to CARLA

### 7. Standard robot_description Package Structure ✅
- **Requirement**: Document what a standard description package looks like
- **Implementation**: Will add detailed section below

## Standard robot_description Package Structure

A robot_description package typically contains URDF/xacro files defining robot geometry, sensors, and kinematics:

```
robot_description/
├── package.xml                          # ROS2 package manifest
├── CMakeLists.txt                       # Build configuration
├── urdf/
│   ├── robot.urdf.xacro                 # Main robot description (xacro macro)
│   ├── robot_base.urdf.xacro            # Base/chassis definition
│   ├── sensors.urdf.xacro               # Sensor definitions
│   └── materials.urdf.xacro             # Visual materials/colors
├── meshes/                              # Visual and collision meshes
│   ├── base_link.stl
│   └── sensor_mount.stl
├── config/
│   └── robot_parameters.yaml            # Robot-specific parameters
└── launch/
    ├── view_robot.launch.py             # Launch rviz to visualize
    └── load_robot.launch.py             # Load to robot_state_publisher
```

### Example URDF with Sensors for CARLA Bridge

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link (vehicle body) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="4.5 1.8 1.5"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Camera Link -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="2.0 0.0 1.5" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- LiDAR Link -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.0 0.0 2.0" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.15"/>
      </geometry>
    </visual>
  </link>

</robot>
```

### How CARLA Bridge Uses robot_description

1. **Loading**:
   - `robot_description` parameter is set by robot_state_publisher or launch file
   - Bridge nodes read the parameter in `on_configure()`

2. **Parsing**:
   - Use urdf library to parse XML
   - Find all links matching sensor patterns (camera_link, lidar_link, etc.)
   - Extract transform from base_link to sensor link

3. **Sensor Spawning**:
   - In `on_activate()`, spawn CARLA sensors at extracted transforms
   - Use link name as frame_id for published data
   - Example:

     ```cpp
auto transform = sensor_parser.getTransform("base_link", "camera_link");
     carla::rpc::Transform carla_transform;
     carla_transform.location.x = transform.translation().x();
     // ... spawn camera at this location
     ```

1. **Benefits**:
   - Single source of truth for robot geometry
   - Consistent TF frames across all nodes
   - Easy to modify sensor placement without changing code
   - Visualization in RViz matches CARLA simulation

### package.xml for Description Package

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_description</name>
  <version>1.0.0</version>
  <description>URDF description of my robot for CARLA simulation</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Launch File to Load Description

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    # Get URDF via xacro
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'robot.urdf.xacro'
    ])

    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([robot_state_publisher])
```

## Updated Package Dependencies

### carla_control/package.xml

```xml
<depend>ackermann_msgs</depend>  <!-- NEW: Use standard message -->
```

### carla_perception/package.xml

```xml
<depend>vision_msgs</depend>  <!-- For Detection2DArray and Detection3DArray -->
```

All dependencies use rosdep keys that can be installed via:

```bash
rosdep install --from-paths src --ignore-src -r -y
```
