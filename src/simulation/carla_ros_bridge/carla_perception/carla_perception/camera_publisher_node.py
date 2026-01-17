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
"""Camera publisher lifecycle node for CARLA with multi-sensor support."""

from dataclasses import dataclass, field
from typing import Optional, Dict, List
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


@dataclass
class CameraConfig:
    """Configuration for a single camera sensor."""

    name: str
    frame_id: str
    image_width: int
    image_height: int
    fov: float
    optical_frame: bool
    # Intrinsics
    camera_matrix: List[float]  # 3x3 = 9 elements, row-major
    distortion_model: str
    distortion_coefficients: List[float]  # typically 5 elements for plumb_bob
    rectification_matrix: List[float]  # 3x3 = 9 elements, row-major
    projection_matrix: List[float]  # 3x4 = 12 elements, row-major


@dataclass
class CameraInstance:
    """Runtime state for a single camera sensor."""

    config: CameraConfig
    sensor: Optional["carla.Sensor"] = None
    image_publisher: Optional[object] = field(default=None)
    camera_info_publisher: Optional[object] = field(default=None)


class CameraPublisherNode(LifecycleNode):
    """Lifecycle node for publishing camera images from CARLA."""

    def __init__(self, node_name="camera_publisher"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("carla_timeout", 10.0)
        self.declare_parameter("role_name", "ego_vehicle")

        # List of camera names to spawn
        self.declare_parameter("camera_names", ["camera"])

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.ego_vehicle: Optional["carla.Vehicle"] = None
        self.cameras: Dict[str, CameraInstance] = {}

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

        # Connect to CARLA
        if carla is None:
            self.get_logger().error("CARLA Python API not available")
            return TransitionCallbackReturn.FAILURE

        try:
            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(timeout)
            version = self.carla_client.get_server_version()
            self.get_logger().info(f"Connected to CARLA {version} at {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        # Find ego vehicle
        try:
            world = self.carla_client.get_world()
            # Wait for a tick to sync with latest world state (needed in synchronous mode)
            world.wait_for_tick()
            actors = world.get_actors()
            vehicles = actors.filter("vehicle.*")

            ego_vehicles = [
                v for v in vehicles if v.attributes.get("role_name") == role_name
            ]
            if not ego_vehicles:
                if vehicles:
                    self.ego_vehicle = vehicles[0]
                    self.get_logger().warn(
                        f'No vehicle with role_name "{role_name}", using first vehicle'
                    )
                else:
                    self.get_logger().error("No vehicles found in CARLA world")
                    return TransitionCallbackReturn.FAILURE
            else:
                self.ego_vehicle = ego_vehicles[0]

            self.get_logger().info(f"Found ego vehicle: {self.ego_vehicle.type_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to find ego vehicle: {e}")
            return TransitionCallbackReturn.FAILURE

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load camera configurations and create publishers
        camera_names = self.get_parameter("camera_names").value
        for name in camera_names:
            config = self._load_camera_config(name)
            if config is None:
                self.get_logger().error(f"Failed to load config for camera '{name}'")
                return TransitionCallbackReturn.FAILURE

            image_publisher = self.create_lifecycle_publisher(
                Image, f"~/{name}/image_raw", 10
            )
            camera_info_publisher = self.create_lifecycle_publisher(
                CameraInfo, f"~/{name}/camera_info", 10
            )
            self.cameras[name] = CameraInstance(
                config=config,
                image_publisher=image_publisher,
                camera_info_publisher=camera_info_publisher,
            )
            self.get_logger().info(
                f"Configured camera '{name}' ({config.image_width}x{config.image_height}) "
                f"with frame_id '{config.frame_id}'"
            )

        self.get_logger().info(f"Configuration complete ({len(self.cameras)} cameras)")
        return TransitionCallbackReturn.SUCCESS

    def _load_camera_config(self, name: str) -> Optional[CameraConfig]:
        """Load configuration for a named camera from parameters."""
        # Declare parameters for this camera
        self.declare_parameter(f"{name}.frame_id", "")
        self.declare_parameter(f"{name}.image_width", 0)
        self.declare_parameter(f"{name}.image_height", 0)
        self.declare_parameter(f"{name}.fov", 90.0)
        self.declare_parameter(f"{name}.optical_frame", True)
        self.declare_parameter(f"{name}.distortion_model", "plumb_bob")
        self.declare_parameter(f"{name}.camera_matrix.data", [0.0] * 9)
        self.declare_parameter(f"{name}.distortion_coefficients.data", [0.0] * 5)
        self.declare_parameter(f"{name}.rectification_matrix.data", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter(f"{name}.projection_matrix.data", [0.0] * 12)

        # Get values
        frame_id = self.get_parameter(f"{name}.frame_id").value
        image_width = self.get_parameter(f"{name}.image_width").value
        image_height = self.get_parameter(f"{name}.image_height").value
        fov = self.get_parameter(f"{name}.fov").value
        optical_frame = self.get_parameter(f"{name}.optical_frame").value
        distortion_model = self.get_parameter(f"{name}.distortion_model").value
        camera_matrix = self.get_parameter(f"{name}.camera_matrix.data").value
        distortion_coefficients = self.get_parameter(f"{name}.distortion_coefficients.data").value
        rectification_matrix = self.get_parameter(f"{name}.rectification_matrix.data").value
        projection_matrix = self.get_parameter(f"{name}.projection_matrix.data").value

        # Validate required fields
        if not frame_id:
            self.get_logger().error(f"Camera '{name}': frame_id is required")
            return None
        if image_width <= 0 or image_height <= 0:
            self.get_logger().error(f"Camera '{name}': image_width and image_height must be positive")
            return None
        if len(camera_matrix) != 9:
            self.get_logger().error(f"Camera '{name}': camera_matrix.data must have 9 elements")
            return None
        if len(projection_matrix) != 12:
            self.get_logger().error(f"Camera '{name}': projection_matrix.data must have 12 elements")
            return None

        return CameraConfig(
            name=name,
            frame_id=frame_id,
            image_width=image_width,
            image_height=image_height,
            fov=fov,
            optical_frame=optical_frame,
            camera_matrix=camera_matrix,
            distortion_model=distortion_model,
            distortion_coefficients=distortion_coefficients,
            rectification_matrix=rectification_matrix,
            projection_matrix=projection_matrix,
        )

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Spawn all camera sensors
        for name, camera in self.cameras.items():
            if not self._spawn_camera(camera):
                self.get_logger().error(f"Failed to spawn camera sensor '{name}'")
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        # Destroy all camera sensors
        for name, camera in self.cameras.items():
            if camera.sensor:
                try:
                    camera.sensor.destroy()
                    camera.sensor = None
                    self.get_logger().info(f"Camera sensor '{name}' destroyed")
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to destroy camera sensor '{name}': {e}"
                    )

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info("Cleaning up...")

        # Destroy publishers
        for name, camera in self.cameras.items():
            if camera.image_publisher:
                self.destroy_publisher(camera.image_publisher)
            if camera.camera_info_publisher:
                self.destroy_publisher(camera.camera_info_publisher)
        self.cameras.clear()

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

    def _spawn_camera(self, camera: CameraInstance) -> bool:
        """Spawn a CARLA camera sensor."""
        try:
            world = self.carla_client.get_world()
            blueprint_library = world.get_blueprint_library()

            # Get camera blueprint
            camera_bp = blueprint_library.find("sensor.camera.rgb")

            # Set camera attributes
            config = camera.config
            camera_bp.set_attribute("image_size_x", str(config.image_width))
            camera_bp.set_attribute("image_size_y", str(config.image_height))
            camera_bp.set_attribute("fov", str(config.fov))

            # Get transform from TF
            camera_transform = self._get_transform_from_tf(
                config.frame_id, config.optical_frame
            )

            # Spawn sensor
            camera.sensor = world.spawn_actor(
                camera_bp, camera_transform, attach_to=self.ego_vehicle
            )

            # Register callback with closure to capture the camera instance
            camera.sensor.listen(lambda data, c=camera: self._camera_callback(data, c))

            self.get_logger().info(
                f"Camera '{config.name}' spawned at {camera_transform.location} "
                f"for frame '{config.frame_id}'"
            )
            return True

        except Exception as e:
            self.get_logger().error(
                f"Failed to spawn camera '{camera.config.name}': {e}"
            )
            return False

    def _get_transform_from_tf(
        self, frame_id: str, optical_frame: bool
    ) -> "carla.Transform":
        """Look up transform from base_link to frame_id and convert to CARLA transform."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", frame_id, rclpy.time.Time()
            )

            # Extract translation
            t = tf.transform.translation
            x, y, z = t.x, t.y, t.z

            # Extract rotation (quaternion to euler)
            q = tf.transform.rotation

            # If optical frame, we need to remove the optical rotation to get physical sensor orientation
            # Optical frame: Z forward, X right, Y down
            # CARLA expects: X forward, Y right, Z up (same as ROS standard)
            if optical_frame:
                # The optical frame has rotation rpy=(-pi/2, 0, -pi/2) applied
                # We need to compute the inverse to get the physical sensor orientation
                # For position, we still use the optical frame position (same as sensor)
                # For rotation, we remove the optical rotation
                return carla.Transform(
                    carla.Location(x=x, y=y, z=z),
                    carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
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
                        roll=math.degrees(roll),
                    ),
                )

        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed for {frame_id}: {e}, using default transform"
            )
            return carla.Transform(
                carla.Location(x=2.0, y=0.0, z=1.5),
                carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
            )

    def _camera_callback(self, carla_image, camera: CameraInstance):
        """Handle incoming camera images from CARLA."""
        if not camera.image_publisher or not camera.image_publisher.is_activated:
            return

        try:
            # Convert CARLA image to ROS Image
            image_msg = Image()
            image_msg.header = Header()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = camera.config.frame_id

            image_msg.height = carla_image.height
            image_msg.width = carla_image.width
            image_msg.encoding = "bgra8"  # CARLA uses BGRA format
            image_msg.step = carla_image.width * 4
            image_msg.is_bigendian = 0

            # Convert raw data to numpy array and then to bytes
            array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
            image_msg.data = array.tobytes()

            # Publish image
            camera.image_publisher.publish(image_msg)

            # Publish camera info
            if (
                camera.camera_info_publisher
                and camera.camera_info_publisher.is_activated
            ):
                camera_info_msg = self._create_camera_info(
                    image_msg.header, camera.config
                )
                camera.camera_info_publisher.publish(camera_info_msg)

        except Exception as e:
            self.get_logger().error(
                f"Error processing camera image for '{camera.config.name}': {e}"
            )

    def _create_camera_info(self, header: Header, config: CameraConfig) -> CameraInfo:
        """Create CameraInfo message from config."""
        camera_info = CameraInfo()
        camera_info.header = header

        camera_info.width = config.image_width
        camera_info.height = config.image_height

        # Camera matrix (K) - 3x3 row-major
        camera_info.k = [float(v) for v in config.camera_matrix]

        # Distortion model and coefficients
        camera_info.distortion_model = config.distortion_model
        camera_info.d = [float(v) for v in config.distortion_coefficients]

        # Rectification matrix (R) - 3x3 row-major
        camera_info.r = [float(v) for v in config.rectification_matrix]

        # Projection matrix (P) - 3x4 row-major
        camera_info.p = [float(v) for v in config.projection_matrix]

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


if __name__ == "__main__":
    main()
