"""LiDAR publisher lifecycle node for CARLA."""
from typing import Optional
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

try:
    import carla
except ImportError:
    carla = None


class LidarPublisherNode(LifecycleNode):
    """Lifecycle node for publishing LiDAR point clouds from CARLA."""

    def __init__(self, node_name='lidar_publisher'):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('role_name', 'ego_vehicle')

        # LiDAR parameters
        self.declare_parameter('channels', 32)
        self.declare_parameter('range', 100.0)
        self.declare_parameter('points_per_second', 56000)
        self.declare_parameter('rotation_frequency', 10.0)
        self.declare_parameter('upper_fov', 10.0)
        self.declare_parameter('lower_fov', -30.0)
        self.declare_parameter('sensor_link', 'lidar_link')
        self.declare_parameter('frame_id', 'lidar_link')

        # State
        self.carla_client: Optional['carla.Client'] = None
        self.ego_vehicle: Optional['carla.Vehicle'] = None
        self.lidar_sensor: Optional['carla.Sensor'] = None

        # ROS interfaces (created in on_configure)
        self.pointcloud_publisher = None

        self.get_logger().info(f'{node_name} initialized')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info('Configuring...')

        # Get parameters
        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        timeout = self.get_parameter('carla_timeout').value
        role_name = self.get_parameter('role_name').value

        # Connect to CARLA
        if carla is None:
            self.get_logger().error('CARLA Python API not available')
            return TransitionCallbackReturn.FAILURE

        try:
            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(timeout)
            version = self.carla_client.get_server_version()
            self.get_logger().info(f'Connected to CARLA {version} at {host}:{port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CARLA: {e}')
            return TransitionCallbackReturn.FAILURE

        # Find ego vehicle
        try:
            world = self.carla_client.get_world()
            actors = world.get_actors()
            vehicles = actors.filter('vehicle.*')

            ego_vehicles = [v for v in vehicles if v.attributes.get('role_name') == role_name]
            if not ego_vehicles:
                if vehicles:
                    self.ego_vehicle = vehicles[0]
                    self.get_logger().warn(f'No vehicle with role_name "{role_name}", using first vehicle')
                else:
                    self.get_logger().error('No vehicles found in CARLA world')
                    return TransitionCallbackReturn.FAILURE
            else:
                self.ego_vehicle = ego_vehicles[0]

            self.get_logger().info(f'Found ego vehicle: {self.ego_vehicle.type_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to find ego vehicle: {e}')
            return TransitionCallbackReturn.FAILURE

        # Create publisher
        self.pointcloud_publisher = self.create_lifecycle_publisher(
            PointCloud2,
            '~/points',
            10
        )

        self.get_logger().info('Configuration complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info('Activating...')

        # Spawn LiDAR sensor
        if not self._spawn_lidar():
            self.get_logger().error('Failed to spawn LiDAR sensor')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Activation complete')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info('Deactivating...')

        # Destroy LiDAR sensor
        if self.lidar_sensor:
            try:
                self.lidar_sensor.destroy()
                self.lidar_sensor = None
                self.get_logger().info('LiDAR sensor destroyed')
            except Exception as e:
                self.get_logger().error(f'Failed to destroy LiDAR sensor: {e}')

        self.get_logger().info('Deactivation complete')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info('Cleaning up...')

        # Destroy publisher
        if self.pointcloud_publisher:
            self.destroy_publisher(self.pointcloud_publisher)
            self.pointcloud_publisher = None

        # Release CARLA resources
        self.ego_vehicle = None
        self.carla_client = None

        self.get_logger().info('Cleanup complete')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def _spawn_lidar(self) -> bool:
        """Spawn CARLA LiDAR sensor."""
        try:
            world = self.carla_client.get_world()
            blueprint_library = world.get_blueprint_library()

            # Get LiDAR blueprint
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

            # Set LiDAR attributes
            channels = self.get_parameter('channels').value
            range_m = self.get_parameter('range').value
            points_per_second = self.get_parameter('points_per_second').value
            rotation_frequency = self.get_parameter('rotation_frequency').value
            upper_fov = self.get_parameter('upper_fov').value
            lower_fov = self.get_parameter('lower_fov').value

            lidar_bp.set_attribute('channels', str(channels))
            lidar_bp.set_attribute('range', str(range_m))
            lidar_bp.set_attribute('points_per_second', str(points_per_second))
            lidar_bp.set_attribute('rotation_frequency', str(rotation_frequency))
            lidar_bp.set_attribute('upper_fov', str(upper_fov))
            lidar_bp.set_attribute('lower_fov', str(lower_fov))

            # TODO: Parse robot_description for sensor placement
            # For now, use default position (on vehicle roof)
            lidar_transform = carla.Transform(
                carla.Location(x=0.0, y=0.0, z=2.0),
                carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            )

            # Spawn sensor
            self.lidar_sensor = world.spawn_actor(
                lidar_bp,
                lidar_transform,
                attach_to=self.ego_vehicle
            )

            # Register callback
            self.lidar_sensor.listen(self._lidar_callback)

            self.get_logger().info(f'LiDAR sensor spawned at {lidar_transform.location}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to spawn LiDAR: {e}')
            return False

    def _lidar_callback(self, carla_lidar_measurement):
        """Handle incoming LiDAR data from CARLA."""
        if not self.pointcloud_publisher or not self.pointcloud_publisher.is_activated:
            return

        try:
            # Convert CARLA LiDAR measurement to ROS PointCloud2
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.get_parameter('frame_id').value

            # Get point cloud data from CARLA
            # CARLA provides points as (x, y, z, intensity)
            points = np.frombuffer(carla_lidar_measurement.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))

            # Create PointCloud2 message
            pointcloud_msg = PointCloud2()
            pointcloud_msg.header = header
            pointcloud_msg.height = 1
            pointcloud_msg.width = points.shape[0]
            pointcloud_msg.is_dense = False
            pointcloud_msg.is_bigendian = False

            # Define point fields
            pointcloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            pointcloud_msg.point_step = 16  # 4 fields * 4 bytes
            pointcloud_msg.row_step = pointcloud_msg.point_step * points.shape[0]
            pointcloud_msg.data = points.tobytes()

            # Publish point cloud
            self.pointcloud_publisher.publish(pointcloud_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')


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


if __name__ == '__main__':
    main()
