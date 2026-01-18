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
"""LiDAR publisher lifecycle node for CARLA with multi-sensor support."""

from dataclasses import dataclass, field
from typing import Optional, Dict
import math
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException

from carla_common import connect_carla, find_ego_vehicle, quaternion_to_euler

try:
    import carla
except ImportError:
    carla = None  # Still needed for type hints and blueprint access


@dataclass
class LidarConfig:
    """Configuration for a single LiDAR sensor."""

    name: str
    frame_id: str
    range: float  # Maximum detection range (meters)
    rotation_frequency: float  # Scan rate (Hz)
    horizontal_fov: float  # Horizontal FOV (degrees), typically 360 for spinning LiDAR
    points_per_channel: int  # Points per channel per rotation (horizontal resolution)
    # Vertical configuration (CARLA distributes channels uniformly between lower/upper FOV)
    channels: int  # Number of laser channels
    upper_fov: float  # Angle in degrees of the highest laser
    lower_fov: float  # Angle in degrees of the lowest laser
    # Dropout configuration (for realistic point loss simulation)
    dropoff_general_rate: float  # Proportion of points randomly dropped (0.0 = none)
    dropoff_intensity_limit: (
        float  # Intensity threshold above which no points are dropped
    )
    dropoff_zero_intensity: float  # Probability of dropping points with zero intensity
    # Atmosphere and noise
    atmosphere_attenuation_rate: float  # Intensity loss coefficient per meter
    noise_stddev: float  # Standard deviation for distance noise (0.0 = no noise)


@dataclass
class LidarInstance:
    """Runtime state for a single LiDAR sensor."""

    config: LidarConfig
    sensor: Optional["carla.Sensor"] = None
    publisher: Optional[object] = field(default=None)
    # Buffer for accumulating points until we have a full rotation
    point_buffer: Optional[np.ndarray] = field(default=None)
    # CARLA frame number when the current rotation started
    rotation_start_frame: Optional[int] = None
    # CARLA timestamp when the current rotation started (for ROS message timestamps)
    rotation_start_timestamp: float = 0.0


class LidarPublisherNode(LifecycleNode):
    """Lifecycle node for publishing LiDAR point clouds from CARLA."""

    def __init__(self, node_name="lidar_publisher"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter(
            "carla_host",
            "localhost",
            ParameterDescriptor(description="CARLA server hostname"),
        )
        self.declare_parameter(
            "carla_port", 2000, ParameterDescriptor(description="CARLA server port")
        )
        self.declare_parameter(
            "carla_timeout",
            10.0,
            ParameterDescriptor(description="Connection timeout in seconds"),
        )
        self.declare_parameter(
            "role_name",
            "ego_vehicle",
            ParameterDescriptor(
                description="Role name of the ego vehicle to attach sensors to"
            ),
        )

        # List of LiDAR names to spawn
        self.declare_parameter(
            "lidar_names",
            ["lidar"],
            ParameterDescriptor(description="List of LiDAR sensor names to spawn"),
        )

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.ego_vehicle: Optional["carla.Vehicle"] = None
        self.lidars: Dict[str, LidarInstance] = {}
        self.simulation_fps: float = 20.0  # Will be updated from CARLA settings

        # TF
        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info("Configuring...")

        # Get parameters
        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        timeout = self.get_parameter("carla_timeout").value
        role_name = self.get_parameter("role_name").value

        try:
            self.carla_client = connect_carla(host, port, timeout)
            self.get_logger().info(f"Connected to CARLA at {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        # Get simulation FPS from CARLA world settings
        try:
            world = self.carla_client.get_world()
            settings = world.get_settings()
            if settings.fixed_delta_seconds and settings.fixed_delta_seconds > 0:
                self.simulation_fps = 1.0 / settings.fixed_delta_seconds
                self.get_logger().info(
                    f"Using CARLA fixed delta time: {settings.fixed_delta_seconds}s "
                    f"({self.simulation_fps:.1f} FPS)"
                )
        except Exception as e:
            self.get_logger().warn(f"Could not get CARLA settings: {e}")

        try:
            world = self.carla_client.get_world()
            world.wait_for_tick()
            self.ego_vehicle = find_ego_vehicle(world, role_name)
            if not self.ego_vehicle:
                self.get_logger().error(
                    f'No vehicle with role_name "{role_name}" found'
                )
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f"Found ego vehicle: {self.ego_vehicle.type_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to find ego vehicle: {e}")
            return TransitionCallbackReturn.FAILURE

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load LiDAR configurations and create publishers
        lidar_names = self.get_parameter("lidar_names").value
        for name in lidar_names:
            try:
                config = self._load_lidar_config(name)
                if config is None:
                    self.get_logger().error(f"Failed to load config for LiDAR '{name}'")
                    return TransitionCallbackReturn.FAILURE

                publisher = self.create_lifecycle_publisher(
                    PointCloud2, f"~/{name}/points", 10
                )
                self.lidars[name] = LidarInstance(config=config, publisher=publisher)
                self.get_logger().info(
                    f"Configured LiDAR '{name}' ({config.channels}ch, {config.range}m range, "
                    f"FOV: {config.lower_fov}° to {config.upper_fov}°) "
                    f"with frame_id '{config.frame_id}'"
                )
            except Exception as e:
                self.get_logger().error(f"Exception loading LiDAR '{name}': {e}")
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"Configuration complete ({len(self.lidars)} LiDARs)")
        return TransitionCallbackReturn.SUCCESS

    def _declare_if_not_exists(self, param_name: str, default_value):
        """Declare a parameter only if it doesn't already exist."""
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, default_value)

    def _load_lidar_config(self, name: str) -> Optional[LidarConfig]:
        """Load configuration for a named LiDAR from parameters."""
        # Declare parameters for this LiDAR (skip if already declared from previous configure)
        self._declare_if_not_exists(f"{name}.frame_id", "")
        self._declare_if_not_exists(f"{name}.range", 0.0)
        self._declare_if_not_exists(f"{name}.rotation_frequency", 0.0)
        self._declare_if_not_exists(f"{name}.horizontal_fov", 360.0)
        self._declare_if_not_exists(f"{name}.points_per_channel", 0)
        # Vertical FOV configuration
        self._declare_if_not_exists(f"{name}.channels", 32)
        self._declare_if_not_exists(f"{name}.upper_fov", 10.0)
        self._declare_if_not_exists(f"{name}.lower_fov", -30.0)
        # Dropout configuration (defaults tuned for minimal dropout like real Velodyne)
        self._declare_if_not_exists(f"{name}.dropoff_general_rate", 0.0)
        self._declare_if_not_exists(f"{name}.dropoff_intensity_limit", 0.8)
        self._declare_if_not_exists(f"{name}.dropoff_zero_intensity", 0.1)
        # Atmosphere and noise
        self._declare_if_not_exists(f"{name}.atmosphere_attenuation_rate", 0.004)
        self._declare_if_not_exists(f"{name}.noise_stddev", 0.0)

        # Get values
        frame_id = self.get_parameter(f"{name}.frame_id").value
        range_m = self.get_parameter(f"{name}.range").value
        rotation_frequency = self.get_parameter(f"{name}.rotation_frequency").value
        horizontal_fov = self.get_parameter(f"{name}.horizontal_fov").value
        points_per_channel = self.get_parameter(f"{name}.points_per_channel").value
        channels = self.get_parameter(f"{name}.channels").value
        upper_fov = self.get_parameter(f"{name}.upper_fov").value
        lower_fov = self.get_parameter(f"{name}.lower_fov").value
        dropoff_general_rate = self.get_parameter(f"{name}.dropoff_general_rate").value
        dropoff_intensity_limit = self.get_parameter(
            f"{name}.dropoff_intensity_limit"
        ).value
        dropoff_zero_intensity = self.get_parameter(
            f"{name}.dropoff_zero_intensity"
        ).value
        atmosphere_attenuation_rate = self.get_parameter(
            f"{name}.atmosphere_attenuation_rate"
        ).value
        noise_stddev = self.get_parameter(f"{name}.noise_stddev").value

        # Validate required fields
        if not frame_id:
            self.get_logger().error(f"LiDAR '{name}': frame_id is required")
            return None
        if range_m <= 0:
            self.get_logger().error(f"LiDAR '{name}': range must be positive")
            return None
        if rotation_frequency <= 0:
            self.get_logger().error(
                f"LiDAR '{name}': rotation_frequency must be positive"
            )
            return None
        if horizontal_fov <= 0:
            self.get_logger().error(f"LiDAR '{name}': horizontal_fov must be positive")
            return None
        if points_per_channel <= 0:
            self.get_logger().error(
                f"LiDAR '{name}': points_per_channel must be positive"
            )
            return None
        if channels <= 0:
            self.get_logger().error(f"LiDAR '{name}': channels must be positive")
            return None
        if upper_fov <= lower_fov:
            self.get_logger().error(
                f"LiDAR '{name}': upper_fov must be greater than lower_fov"
            )
            return None

        return LidarConfig(
            name=name,
            frame_id=frame_id,
            range=range_m,
            rotation_frequency=rotation_frequency,
            horizontal_fov=horizontal_fov,
            points_per_channel=points_per_channel,
            channels=channels,
            upper_fov=upper_fov,
            lower_fov=lower_fov,
            dropoff_general_rate=dropoff_general_rate,
            dropoff_intensity_limit=dropoff_intensity_limit,
            dropoff_zero_intensity=dropoff_zero_intensity,
            atmosphere_attenuation_rate=atmosphere_attenuation_rate,
            noise_stddev=noise_stddev,
        )

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Spawn all LiDAR sensors
        for name, lidar in self.lidars.items():
            if not self._spawn_lidar(lidar):
                self.get_logger().error(f"Failed to spawn LiDAR sensor '{name}'")
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        # Destroy all LiDAR sensors
        for name, lidar in self.lidars.items():
            if lidar.sensor:
                try:
                    lidar.sensor.destroy()
                    lidar.sensor = None
                    self.get_logger().info(f"LiDAR sensor '{name}' destroyed")
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to destroy LiDAR sensor '{name}': {e}"
                    )

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info("Cleaning up...")

        # Destroy publishers
        for name, lidar in self.lidars.items():
            if lidar.publisher:
                self.destroy_publisher(lidar.publisher)
        self.lidars.clear()

        # Release CARLA resources
        self.ego_vehicle = None
        self.carla_client = None

        # Release TF resources
        self.tf_listener = None
        self.tf_buffer = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def _spawn_lidar(self, lidar: LidarInstance) -> bool:
        """Spawn a CARLA LiDAR sensor."""
        try:
            world = self.carla_client.get_world()
            blueprint_library = world.get_blueprint_library()

            # Get LiDAR blueprint
            lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")

            # Set LiDAR attributes
            config = lidar.config

            # Calculate points_per_second from typical LiDAR specs
            # points_per_second = points_per_channel * channels * rotation_frequency
            points_per_second = int(
                config.points_per_channel * config.channels * config.rotation_frequency
            )

            # Core LiDAR geometry
            lidar_bp.set_attribute("channels", str(config.channels))
            lidar_bp.set_attribute("range", str(config.range))
            lidar_bp.set_attribute("points_per_second", str(points_per_second))
            lidar_bp.set_attribute("rotation_frequency", str(config.rotation_frequency))
            lidar_bp.set_attribute("upper_fov", str(config.upper_fov))
            lidar_bp.set_attribute("lower_fov", str(config.lower_fov))
            lidar_bp.set_attribute("horizontal_fov", str(config.horizontal_fov))

            # Dropout configuration (for realistic point loss)
            lidar_bp.set_attribute(
                "dropoff_general_rate", str(config.dropoff_general_rate)
            )
            lidar_bp.set_attribute(
                "dropoff_intensity_limit", str(config.dropoff_intensity_limit)
            )
            lidar_bp.set_attribute(
                "dropoff_zero_intensity", str(config.dropoff_zero_intensity)
            )

            # Atmosphere and noise
            lidar_bp.set_attribute(
                "atmosphere_attenuation_rate", str(config.atmosphere_attenuation_rate)
            )
            lidar_bp.set_attribute("noise_stddev", str(config.noise_stddev))

            # Get transform from TF
            lidar_transform = self._get_transform_from_tf(config.frame_id)
            if lidar_transform is None:
                return False

            # Spawn sensor
            lidar.sensor = world.spawn_actor(
                lidar_bp, lidar_transform, attach_to=self.ego_vehicle
            )

            # Register callback with closure to capture the lidar instance
            lidar.sensor.listen(
                lambda data, lidar_inst=lidar: self._lidar_callback(data, lidar_inst)
            )

            self.get_logger().info(
                f"LiDAR '{config.name}' spawned at {lidar_transform.location} "
                f"for frame '{config.frame_id}'"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to spawn LiDAR '{lidar.config.name}': {e}")
            return False

    def _get_transform_from_tf(self, frame_id: str) -> Optional["carla.Transform"]:
        """Look up transform from base_link to frame_id and convert to CARLA transform."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", frame_id, rclpy.time.Time()
            )

            t = tf.transform.translation
            x, y, z = t.x, t.y, t.z

            q = tf.transform.rotation
            roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

            return carla.Transform(
                carla.Location(x=x, y=y, z=z),
                carla.Rotation(
                    pitch=math.degrees(pitch),
                    yaw=math.degrees(yaw),
                    roll=math.degrees(roll),
                ),
            )

        except TransformException as e:
            self.get_logger().error(f"TF lookup failed for {frame_id}: {e}")
            return None

    def _lidar_callback(self, carla_lidar_measurement, lidar: LidarInstance):
        """Handle incoming LiDAR data from CARLA.

        Uses frame-based accumulation for reliable point cloud synchronization.
        The CARLA 'frame' attribute is a monotonically increasing integer that
        identifies the simulation step, making it ideal for accumulating readings
        across multiple callbacks within a single rotation period.
        """
        if not lidar.publisher or not lidar.publisher.is_activated:
            return

        try:
            # Get point cloud data from CARLA
            # CARLA provides points as (x, y, z, intensity)
            points = np.frombuffer(
                carla_lidar_measurement.raw_data, dtype=np.dtype("f4")
            )
            points = np.reshape(points, (int(points.shape[0] / 4), 4)).copy()

            # Flip Y axis: CARLA uses left-handed coordinates, ROS uses right-handed
            points[:, 1] *= -1

            current_frame = carla_lidar_measurement.frame
            current_timestamp = carla_lidar_measurement.timestamp

            # Calculate how many frames constitute one full LiDAR rotation
            # Use ceil to ensure we accumulate enough frames for a complete rotation
            # (truncating with int() would publish before rotation completes)
            frames_per_rotation = math.ceil(
                self.simulation_fps / lidar.config.rotation_frequency
            )

            # Frame-based accumulation logic
            if lidar.rotation_start_frame is not None:
                frames_elapsed = current_frame - lidar.rotation_start_frame
                if frames_elapsed >= frames_per_rotation:
                    # Full rotation complete - publish accumulated point cloud
                    self._publish_pointcloud(
                        lidar, lidar.point_buffer, lidar.rotation_start_timestamp
                    )
                    # Start new rotation with current points
                    lidar.point_buffer = points
                    lidar.rotation_start_frame = current_frame
                    lidar.rotation_start_timestamp = current_timestamp
                else:
                    # Accumulate points within the same rotation
                    lidar.point_buffer = np.vstack([lidar.point_buffer, points])
            else:
                # First callback - initialize buffer and start frame
                lidar.point_buffer = points
                lidar.rotation_start_frame = current_frame
                lidar.rotation_start_timestamp = current_timestamp

        except Exception as e:
            self.get_logger().error(
                f"Error processing LiDAR data for '{lidar.config.name}': {e}"
            )

    def _publish_pointcloud(
        self, lidar: LidarInstance, points: np.ndarray, carla_timestamp: float
    ):
        """Publish accumulated point cloud."""
        header = Header()
        # Convert CARLA timestamp (seconds since episode start) to ROS time
        sec = int(carla_timestamp)
        nanosec = int((carla_timestamp - sec) * 1e9)
        header.stamp.sec = sec
        header.stamp.nanosec = nanosec
        header.frame_id = lidar.config.frame_id

        # Create PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = points.shape[0]
        pointcloud_msg.is_dense = False
        pointcloud_msg.is_bigendian = False

        # Define point fields
        pointcloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        pointcloud_msg.point_step = 16  # 4 fields * 4 bytes
        pointcloud_msg.row_step = pointcloud_msg.point_step * points.shape[0]
        pointcloud_msg.data = points.tobytes()

        # Publish point cloud
        lidar.publisher.publish(pointcloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
