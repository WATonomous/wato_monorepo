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
"""Bounding box publisher lifecycle node for CARLA."""

from typing import Optional
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, PoseStamped
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

from carla_common import (
    connect_carla,
    find_ego_vehicle,
    euler_to_quaternion,
    carla_to_ros_position,
    carla_to_ros_rotation,
)

try:
    import carla
except ImportError:
    carla = None  # Still needed for type hints


class BBoxPublisherNode(LifecycleNode):
    """Lifecycle node for publishing 2D and 3D bounding boxes from CARLA."""

    def __init__(self, node_name="bbox_publisher"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter(
            "carla_host",
            "localhost",
            ParameterDescriptor(description="CARLA server hostname"),
        )
        self.declare_parameter(
            "carla_port",
            2000,
            ParameterDescriptor(description="CARLA server port"),
        )
        self.declare_parameter(
            "carla_timeout",
            10.0,
            ParameterDescriptor(description="Connection timeout in seconds"),
        )
        self.declare_parameter(
            "role_name",
            "ego_vehicle",
            ParameterDescriptor(description="Role name of the ego vehicle"),
        )

        # Publishing parameters
        self.declare_parameter(
            "publish_rate",
            10.0,
            ParameterDescriptor(description="Detection publish rate in Hz"),
        )
        self.declare_parameter(
            "frame_id",
            "base_link",
            ParameterDescriptor(description="TF frame for published detections"),
        )
        self.declare_parameter(
            "include_vehicles",
            True,
            ParameterDescriptor(description="Include vehicles in detections"),
        )
        self.declare_parameter(
            "include_pedestrians",
            True,
            ParameterDescriptor(description="Include pedestrians in detections"),
        )
        self.declare_parameter(
            "max_distance",
            -1.0,
            ParameterDescriptor(
                description="Max distance from ego vehicle in meters (-1 = no limit)"
            ),
        )

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.world: Optional["carla.World"] = None
        self.ego_vehicle: Optional["carla.Vehicle"] = None

        # ROS interfaces (created in on_configure)
        self.detections_2d_publisher = None
        self.detections_3d_publisher = None
        self.tracked_detections_3d_publisher = None
        self.publish_timer = None

        # TF2 for frame transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info("Configuring...")

        # Get parameters
        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        timeout = self.get_parameter("carla_timeout").value

        # Connect to CARLA
        try:
            self.carla_client = connect_carla(host, port, timeout)
            self.get_logger().info(f"Connected to CARLA at {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        # Find ego vehicle
        role_name = self.get_parameter("role_name").value
        try:
            self.world = self.carla_client.get_world()
            self.world.wait_for_tick()
            self.ego_vehicle = find_ego_vehicle(self.world, role_name)
            if not self.ego_vehicle:
                self.get_logger().error(
                    f'No vehicle with role_name "{role_name}" found'
                )
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f"Found ego vehicle: {self.ego_vehicle.type_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to find ego vehicle: {e}")
            return TransitionCallbackReturn.FAILURE

        # Create publishers
        self.detections_2d_publisher = self.create_lifecycle_publisher(
            Detection2DArray, "detections_2d", 10
        )

        self.detections_3d_publisher = self.create_lifecycle_publisher(
            Detection3DArray, "detections_3d", 10
        )

        self.tracked_detections_3d_publisher = self.create_lifecycle_publisher(
            Detection3DArray, "tracked_detections_3d", 10
        )

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Create timer for publishing detections
        publish_rate = self.get_parameter("publish_rate").value
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_timer_callback
        )

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        # Stop publish timer
        if self.publish_timer:
            self.destroy_timer(self.publish_timer)
            self.publish_timer = None

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info("Cleaning up...")

        # Destroy publishers
        if self.detections_2d_publisher:
            self.destroy_publisher(self.detections_2d_publisher)
            self.detections_2d_publisher = None

        if self.detections_3d_publisher:
            self.destroy_publisher(self.detections_3d_publisher)
            self.detections_3d_publisher = None

        if self.tracked_detections_3d_publisher:
            self.destroy_publisher(self.tracked_detections_3d_publisher)
            self.tracked_detections_3d_publisher = None

        # Release CARLA resources
        self.ego_vehicle = None
        self.world = None
        self.carla_client = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def publish_timer_callback(self):
        """Periodically query CARLA for actors and publish bounding boxes."""
        if not self.world or not self.ego_vehicle:
            return

        if not self.detections_3d_publisher:
            return

        try:
            frame_id = self.get_parameter("frame_id").value
            stamp = self.get_clock().now().to_msg()

            # Get all actors from CARLA world
            actors = self.world.get_actors()

            # Filter actors based on parameters
            include_vehicles = self.get_parameter("include_vehicles").value
            include_pedestrians = self.get_parameter("include_pedestrians").value
            max_distance = self.get_parameter("max_distance").value

            filtered_actors = []
            if include_vehicles:
                filtered_actors.extend(actors.filter("vehicle.*"))
            if include_pedestrians:
                filtered_actors.extend(actors.filter("walker.pedestrian.*"))

            # Filter by distance if max_distance is set
            if max_distance > 0:
                ego_location = self.ego_vehicle.get_location()
                filtered_actors = [
                    actor
                    for actor in filtered_actors
                    if actor.get_location().distance(ego_location) <= max_distance
                ]

            # Create header
            header = Header()
            header.stamp = stamp
            header.frame_id = frame_id

            # Create 3D detections arrays
            detections_3d = Detection3DArray()
            detections_3d.header = header

            tracked_detections_3d = Detection3DArray()
            tracked_detections_3d.header = header

            # Check if we need TF transform (frame_id is not "map")
            transform = None
            if frame_id != "map":
                try:
                    # Use Time() to get latest available transform rather than exact timestamp
                    transform = self.tf_buffer.lookup_transform(
                        frame_id,
                        "map",
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                except TransformException as e:
                    self.get_logger().warn(
                        f"Could not get transform from map to {frame_id}: {e}"
                    )
                    return

            for actor in filtered_actors:
                # Skip ego vehicle
                if actor.id == self.ego_vehicle.id:
                    continue
                detection_3d = self._create_detection_3d(actor, stamp, transform)
                if detection_3d:
                    detections_3d.detections.append(detection_3d)

                    # Create tracked detection with actor ID for tracking
                    tracked_detection = self._create_detection_3d(
                        actor, stamp, transform, track_id=str(actor.id)
                    )
                    if tracked_detection:
                        tracked_detections_3d.detections.append(tracked_detection)

            # Publish 3D detections
            self.detections_3d_publisher.publish(detections_3d)

            # Publish tracked 3D detections
            if self.tracked_detections_3d_publisher:
                self.tracked_detections_3d_publisher.publish(tracked_detections_3d)

        except Exception as e:
            self.get_logger().error(f"Error publishing detections: {e}")

    def _create_detection_3d(
        self, actor: "carla.Actor", stamp, transform, track_id: Optional[str] = None
    ) -> Optional[Detection3D]:
        """Create Detection3D message from CARLA actor.

        Coordinates are first computed in map frame (CARLA world -> ROS), then
        transformed to the target frame using TF2 if transform is provided.

        Args:
            actor: CARLA actor to create detection for.
            stamp: ROS timestamp for the detection.
            transform: TF2 transform to apply (None if frame_id is "map").
            track_id: Optional tracking ID for the detection (e.g., actor ID).
        """
        try:
            detection = Detection3D()

            # Set tracking ID if provided
            if track_id is not None:
                detection.id = track_id

            # Get actor's bounding box and transform
            bbox = actor.bounding_box
            actor_transform = actor.get_transform()

            # Convert CARLA transform to ROS coordinates
            # CARLA vehicle location is at ground level, so offset z by half bbox height
            # Pedestrians already have their origin at center mass, so no offset needed
            is_vehicle = "vehicle" in actor.type_id
            ros_x, ros_y, ros_z = carla_to_ros_position(
                actor_transform.location.x,
                actor_transform.location.y,
                actor_transform.location.z + (bbox.extent.z if is_vehicle else 0.0),
            )
            roll, pitch, yaw = carla_to_ros_rotation(
                actor_transform.rotation.roll,
                actor_transform.rotation.pitch,
                actor_transform.rotation.yaw,
            )
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

            # If we have a transform, apply it to get coordinates in target frame
            if transform is not None:
                # Create PoseStamped in map frame
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = stamp
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position = Point(x=ros_x, y=ros_y, z=ros_z)
                pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

                # Transform to target frame
                transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_stamped, transform
                )

                # Use transformed coordinates
                detection.bbox.center = transformed_pose.pose
            else:
                # No transform needed, use map frame coordinates directly
                detection.bbox.center = Pose()
                detection.bbox.center.position = Point(x=ros_x, y=ros_y, z=ros_z)
                detection.bbox.center.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            # Set bbox size (CARLA bbox extent is half-size)
            detection.bbox.size = Vector3(
                x=bbox.extent.x * 2.0, y=bbox.extent.y * 2.0, z=bbox.extent.z * 2.0
            )

            # Set classification
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self._get_class_id(actor.type_id)
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)

            return detection

        except Exception as e:
            self.get_logger().error(
                f"Error creating detection for actor {actor.id}: {e}"
            )
            return None

    def _get_class_id(self, type_id: str) -> str:
        """Map CARLA type_id to class identifier."""
        if "vehicle" in type_id:
            if "car" in type_id:
                return "car"
            elif "truck" in type_id:
                return "truck"
            elif "bike" in type_id or "bicycle" in type_id:
                return "bicycle"
            elif "motorcycle" in type_id:
                return "motorcycle"
            else:
                return "vehicle"
        elif "pedestrian" in type_id:
            return "pedestrian"
        else:
            return "unknown"


def main(args=None):
    rclpy.init(args=args)
    node = BBoxPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
