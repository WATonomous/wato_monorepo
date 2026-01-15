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
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header

try:
    import carla
except ImportError:
    carla = None


class BBoxPublisherNode(LifecycleNode):
    """Lifecycle node for publishing 2D and 3D bounding boxes from CARLA."""

    def __init__(self, node_name="bbox_publisher"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("carla_timeout", 10.0)

        # Publishing parameters
        self.declare_parameter("publish_rate", 10.0)  # Hz
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("include_vehicles", True)
        self.declare_parameter("include_pedestrians", True)

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.world: Optional["carla.World"] = None

        # ROS interfaces (created in on_configure)
        self.detections_2d_publisher = None
        self.detections_3d_publisher = None
        self.publish_timer = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info("Configuring...")

        # Get parameters
        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        timeout = self.get_parameter("carla_timeout").value

        # Connect to CARLA
        if carla is None:
            self.get_logger().error("CARLA Python API not available")
            return TransitionCallbackReturn.FAILURE

        try:
            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(timeout)
            self.world = self.carla_client.get_world()
            version = self.carla_client.get_server_version()
            self.get_logger().info(f"Connected to CARLA {version} at {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        # Create publishers
        self.detections_2d_publisher = self.create_lifecycle_publisher(
            Detection2DArray, "~/detections_2d", 10
        )

        self.detections_3d_publisher = self.create_lifecycle_publisher(
            Detection3DArray, "~/detections_3d", 10
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

        # Release CARLA resources
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
        if not self.world:
            return

        if (
            not self.detections_3d_publisher
            or not self.detections_3d_publisher.is_activated
        ):
            return

        try:
            # Get all actors from CARLA world
            actors = self.world.get_actors()

            # Filter actors based on parameters
            include_vehicles = self.get_parameter("include_vehicles").value
            include_pedestrians = self.get_parameter("include_pedestrians").value

            filtered_actors = []
            if include_vehicles:
                filtered_actors.extend(actors.filter("vehicle.*"))
            if include_pedestrians:
                filtered_actors.extend(actors.filter("walker.pedestrian.*"))

            # Create header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.get_parameter("frame_id").value

            # Create 3D detections
            detections_3d = Detection3DArray()
            detections_3d.header = header

            for actor in filtered_actors:
                detection_3d = self._create_detection_3d(actor)
                if detection_3d:
                    detections_3d.detections.append(detection_3d)

            # Publish 3D detections
            self.detections_3d_publisher.publish(detections_3d)

            # TODO: Create and publish 2D detections (requires camera projection)
            # For now, we'll skip 2D detections
            # detections_2d = Detection2DArray()
            # detections_2d.header = header
            # self.detections_2d_publisher.publish(detections_2d)

        except Exception as e:
            self.get_logger().error(f"Error publishing detections: {e}")

    def _create_detection_3d(self, actor: "carla.Actor") -> Optional[Detection3D]:
        """Create Detection3D message from CARLA actor."""
        try:
            detection = Detection3D()

            # Get actor's bounding box
            bbox = actor.bounding_box
            transform = actor.get_transform()

            # Set bbox center pose
            detection.bbox.center = Pose()
            detection.bbox.center.position = Point(
                x=transform.location.x, y=transform.location.y, z=transform.location.z
            )

            # Convert rotation to quaternion
            # CARLA uses pitch-yaw-roll, convert to quaternion
            import math

            pitch = math.radians(transform.rotation.pitch)
            yaw = math.radians(transform.rotation.yaw)
            roll = math.radians(transform.rotation.roll)

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            detection.bbox.center.orientation = Quaternion(
                w=cr * cp * cy + sr * sp * sy,
                x=sr * cp * cy - cr * sp * sy,
                y=cr * sp * cy + sr * cp * sy,
                z=cr * cp * sy - sr * sp * cy,
            )

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
