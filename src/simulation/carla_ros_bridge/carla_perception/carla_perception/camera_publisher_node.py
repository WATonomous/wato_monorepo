"""Camera publisher lifecycle node for CARLA."""
from typing import Optional
import math
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException

try:
    import carla
except ImportError:
    carla = None


class CameraPublisherNode(LifecycleNode):
    """Lifecycle node for publishing camera images from CARLA."""

    def __init__(self, node_name='camera_publisher'):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('role_name', 'ego_vehicle')

        # Camera parameters
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 600)
        self.declare_parameter('fov', 90.0)
        self.declare_parameter('frame_id', 'front_camera_optical')
        self.declare_parameter('optical_frame', True)

        # State
        self.carla_client: Optional['carla.Client'] = None
        self.ego_vehicle: Optional['carla.Vehicle'] = None
        self.camera_sensor: Optional['carla.Sensor'] = None

        # TF
        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None

        # ROS interfaces (created in on_configure)
        self.image_publisher = None
        self.camera_info_publisher = None

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

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publishers
        self.image_publisher = self.create_lifecycle_publisher(
            Image,
            '~/image_raw',
            10
        )

        self.camera_info_publisher = self.create_lifecycle_publisher(
            CameraInfo,
            '~/camera_info',
            10
        )

        self.get_logger().info('Configuration complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info('Activating...')

        # Spawn camera sensor
        if not self._spawn_camera():
            self.get_logger().error('Failed to spawn camera sensor')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Activation complete')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info('Deactivating...')

        # Destroy camera sensor
        if self.camera_sensor:
            try:
                self.camera_sensor.destroy()
                self.camera_sensor = None
                self.get_logger().info('Camera sensor destroyed')
            except Exception as e:
                self.get_logger().error(f'Failed to destroy camera sensor: {e}')

        self.get_logger().info('Deactivation complete')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info('Cleaning up...')

        # Destroy publishers
        if self.image_publisher:
            self.destroy_publisher(self.image_publisher)
            self.image_publisher = None

        if self.camera_info_publisher:
            self.destroy_publisher(self.camera_info_publisher)
            self.camera_info_publisher = None

        # Release CARLA resources
        self.ego_vehicle = None
        self.carla_client = None

        # Release TF resources
        self.tf_listener = None
        self.tf_buffer = None

        self.get_logger().info('Cleanup complete')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def _spawn_camera(self) -> bool:
        """Spawn CARLA camera sensor."""
        try:
            world = self.carla_client.get_world()
            blueprint_library = world.get_blueprint_library()

            # Get camera blueprint
            camera_bp = blueprint_library.find('sensor.camera.rgb')

            # Set camera attributes
            width = self.get_parameter('image_width').value
            height = self.get_parameter('image_height').value
            fov = self.get_parameter('fov').value

            camera_bp.set_attribute('image_size_x', str(width))
            camera_bp.set_attribute('image_size_y', str(height))
            camera_bp.set_attribute('fov', str(fov))

            # Get transform from TF
            frame_id = self.get_parameter('frame_id').value
            camera_transform = self._get_transform_from_tf(frame_id)

            # Spawn sensor
            self.camera_sensor = world.spawn_actor(
                camera_bp,
                camera_transform,
                attach_to=self.ego_vehicle
            )

            # Register callback
            self.camera_sensor.listen(self._camera_callback)

            self.get_logger().info(f'Camera sensor spawned at {camera_transform.location} for frame {frame_id}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to spawn camera: {e}')
            return False

    def _get_transform_from_tf(self, frame_id: str) -> 'carla.Transform':
        """Look up transform from base_link to frame_id and convert to CARLA transform."""
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame_id, rclpy.time.Time())

            # Extract translation
            t = tf.transform.translation
            x, y, z = t.x, t.y, t.z

            # Extract rotation (quaternion to euler)
            q = tf.transform.rotation

            # If optical frame, we need to remove the optical rotation to get physical sensor orientation
            # Optical frame: Z forward, X right, Y down
            # CARLA expects: X forward, Y right, Z up (same as ROS standard)
            if self.get_parameter('optical_frame').value:
                # The optical frame has rotation rpy=(-pi/2, 0, -pi/2) applied
                # We need to compute the inverse to get the physical sensor orientation
                # For position, we still use the optical frame position (same as sensor)
                # For rotation, we remove the optical rotation
                return carla.Transform(
                    carla.Location(x=x, y=y, z=z),
                    carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                )
            else:
                # Non-optical frame: convert quaternion to euler directly
                sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
                cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                roll = math.atan2(sinr_cosp, cosr_cosp)

                sinp = 2.0 * (q.w * q.y - q.z * q.x)
                if abs(sinp) >= 1:
                    pitch = math.copysign(math.pi / 2, sinp)
                else:
                    pitch = math.asin(sinp)

                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                return carla.Transform(
                    carla.Location(x=x, y=y, z=z),
                    carla.Rotation(
                        pitch=math.degrees(pitch),
                        yaw=math.degrees(yaw),
                        roll=math.degrees(roll)
                    )
                )

        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed for {frame_id}: {e}, using default transform')
            return carla.Transform(
                carla.Location(x=2.0, y=0.0, z=1.5),
                carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            )

    def _camera_callback(self, carla_image):
        """Handle incoming camera images from CARLA."""
        if not self.image_publisher or not self.image_publisher.is_activated:
            return

        try:
            # Convert CARLA image to ROS Image
            image_msg = Image()
            image_msg.header = Header()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.get_parameter('frame_id').value

            image_msg.height = carla_image.height
            image_msg.width = carla_image.width
            image_msg.encoding = 'bgra8'  # CARLA uses BGRA format
            image_msg.step = carla_image.width * 4
            image_msg.is_bigendian = 0

            # Convert raw data to numpy array and then to bytes
            array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
            image_msg.data = array.tobytes()

            # Publish image
            self.image_publisher.publish(image_msg)

            # Publish camera info
            if self.camera_info_publisher and self.camera_info_publisher.is_activated:
                camera_info_msg = self._create_camera_info(image_msg.header)
                self.camera_info_publisher.publish(camera_info_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def _create_camera_info(self, header: Header) -> CameraInfo:
        """Create CameraInfo message."""
        camera_info = CameraInfo()
        camera_info.header = header

        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        fov = self.get_parameter('fov').value

        # Calculate focal length from FOV
        focal_length = width / (2.0 * math.tan(math.radians(fov) / 2.0))

        camera_info.width = width
        camera_info.height = height

        # Camera matrix (K)
        camera_info.k = [
            focal_length, 0.0, width / 2.0,
            0.0, focal_length, height / 2.0,
            0.0, 0.0, 1.0
        ]

        # Distortion coefficients (assuming no distortion)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.distortion_model = 'plumb_bob'

        # Rectification matrix (identity for monocular camera)
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix
        camera_info.p = [
            focal_length, 0.0, width / 2.0, 0.0,
            0.0, focal_length, height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return camera_info


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
