import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from tracking_msgs.msg import TrackedObstacleList  # Adjust based on your custom message type
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import tf2_ros
import cv2
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf_transformations

class ObjectTrackingVisualizer(Node):
    def __init__(self):
        super().__init__("object_tracking_visualizer")

        # Subscribing to the tracked obstacles topic
        self.tracked_obstacles_sub = self.create_subscription(
            TrackedObstacleList,
            "/tracked_obstacles",
            self.tracked_obstacles_callback,
            10
        )

        # Subscribing to the camera image feed
        self.image_sub = self.create_subscription(
            CompressedImage, "/CAM_FRONT/image_rect_compressed", self.image_callback, 10
        )

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/tracked_objects_markers", 10)
        self.image_pub = self.create_publisher(Image, "/annotated_image", 10)

        # Namespace for the markers
        self.marker_namespace = "tracked_objects"

        # For image handling
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_tracked_obstacles = None

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def image_callback(self, msg):
        """Store the latest image frame."""
        try:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to decode compressed image: {str(e)}")

    def tracked_obstacles_callback(self, msg):
        """Process the tracked obstacles to generate markers and update the image."""
        self.latest_tracked_obstacles = msg

        # Generate 3D visualization markers for bounding boxes
        self.generate_markers(msg)

        # Annotate the camera image with bounding boxes and IDs
        self.annotate_image_with_bounding_boxes()

    def generate_markers(self, msg):
        """Generate markers for visualizing bounding boxes in 3D."""
        marker_array = MarkerArray()
        for tracked_obstacle in msg.tracked_obstacles:
            obj_id = tracked_obstacle.obstacle.object_id
            pose = tracked_obstacle.obstacle.pose.pose
            dimensions = tracked_obstacle.obstacle

            # Transform pose from map to base_link
            transformed_pose = self.transform_pose(pose, "map", "base_link")

            # Create a CUBE marker for the bounding box
            box_marker = Marker()
            box_marker.header.frame_id = "base_link"  # Ensure consistent frame
            box_marker.header.stamp = self.get_clock().now().to_msg()
            box_marker.ns = f"{self.marker_namespace}_box"
            box_marker.id = obj_id
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose = transformed_pose
            box_marker.scale.x = dimensions.width_along_x_axis
            box_marker.scale.y = dimensions.height_along_y_axis
            box_marker.scale.z = dimensions.depth_along_z_axis
            box_marker.color.r = 1.0
            box_marker.color.g = 1.0
            box_marker.color.b = 0.0
            box_marker.color.a = 0.8  # Transparency

            # Add the box marker to the array
            marker_array.markers.append(box_marker)

            # Create a TEXT_VIEW_FACING marker for the object ID
            text_marker = Marker()
            text_marker.header.frame_id = "base_link"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f"{self.marker_namespace}_id"
            text_marker.id = obj_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = transformed_pose
            text_marker.pose.position.z += dimensions.depth_along_z_axis / 2.0 + 0.5  # Adjust text position above the box
            text_marker.scale.z = 0.5  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = f"ID: {obj_id}"

            # Add the text marker to the array
            marker_array.markers.append(text_marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def transform_pose(self, pose, from_frame, to_frame):
        """Transform pose from one frame to another."""
        try:
            # Look up the transformation
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose

            # Apply the transformation manually
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Transform the position
            transformed_position = [
                pose.position.x + translation.x,
                pose.position.y + translation.y,
                pose.position.z + translation.z,
            ]

            # Transform the orientation
            rotation_matrix = tf_transformations.quaternion_matrix(
                [rotation.x, rotation.y, rotation.z, rotation.w]
            )
            transformed_orientation = tf_transformations.quaternion_from_matrix(rotation_matrix)

            transformed_pose = PoseStamped()
            transformed_pose.pose.position.x = transformed_position[0]
            transformed_pose.pose.position.y = transformed_position[1]
            transformed_pose.pose.position.z = transformed_position[2]
            transformed_pose.pose.orientation.x = transformed_orientation[0]
            transformed_pose.pose.orientation.y = transformed_orientation[1]
            transformed_pose.pose.orientation.z = transformed_orientation[2]
            transformed_pose.pose.orientation.w = transformed_orientation[3]

            return transformed_pose.pose
        except Exception as e:
            self.get_logger().error(f"Failed to transform pose from {from_frame} to {to_frame}: {str(e)}")
            return pose  # Return the original pose as a fallback

    def annotate_image_with_bounding_boxes(self):
        """Annotate the latest image with bounding boxes and IDs."""
        if self.latest_image is None or self.latest_tracked_obstacles is None:
            return  # Wait until both the image and tracked obstacles are available

        for tracked_obstacle in self.latest_tracked_obstacles.tracked_obstacles:
            obj_id = tracked_obstacle.obstacle.object_id
            pose = tracked_obstacle.obstacle.pose.pose
            dimensions = tracked_obstacle.obstacle

            # Placeholder projection logic for 3D to 2D coordinates
            top_left = (int(pose.position.x), int(pose.position.y))
            bottom_right = (
                int(pose.position.x + dimensions.width_along_x_axis),
                int(pose.position.y + dimensions.height_along_y_axis),
            )

            # Draw the bounding box
            cv2.rectangle(self.latest_image, top_left, bottom_right, (0, 255, 0), 2)

            # Annotate with the ID
            text = f"ID: {obj_id}"
            cv2.putText(self.latest_image, text, (top_left[0], top_left[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Convert the annotated image back to a ROS image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(self.latest_image, encoding="bgr8")

        # Publish the annotated image
        self.image_pub.publish(annotated_image_msg)


def main(args=None):
    rclpy.init(args=args)
    visualizer = ObjectTrackingVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
