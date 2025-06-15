import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
# Update this if your custom message name differs
from tracking_msgs.msg import TrackedObstacleList
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class ObjectTrackingVisualizer(Node):
    def __init__(self):
        super().__init__("object_tracking_visualizer")
        # Subscribing to the tracked obstacles topic
        self.tracked_obstacles_sub = self.create_subscription(
            TrackedObstacleList,  # Message type of the tracked obstacles
            "/tracked_obstacles",  # Topic name
            self.tracked_obstacles_callback,
            10  # Queue size
        )
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, "/tracked_objects_markers", 10)
        # Subscribing to the camera image feed
        self.image_sub = self.create_subscription(
            CompressedImage, "/CAM_FRONT/image_rect_compressed", self.image_callback, 10
        )
        # Publisher for the annotated image
        self.image_pub = self.create_publisher(Image, "/annotated_image", 10)
        # Namespace for the markers
        self.marker_namespace = "tracked_objects"
        # For image handling
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_tracked_obstacles = None

    def image_callback(self, msg: CompressedImage):
        """Store and convert the latest compressed image frame."""
        try:
            # Convert the compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Convert the OpenCV image to a ROS Image message
            self.latest_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to decode compressed image: {e}")

    def tracked_obstacles_callback(self, msg: TrackedObstacleList):
        """Process the tracked obstacles to generate markers and update the image."""
        self.latest_tracked_obstacles = msg
        # Generate 3D visualization markers for bounding boxes
        self.generate_markers(msg)
        # Annotate the camera image with bounding boxes and IDs
        self.annotate_image_with_bounding_boxes()

    def generate_markers(self, msg: TrackedObstacleList):
        """Generate markers for visualizing bounding boxes in 3D."""
        marker_array = MarkerArray()
        for tracked_obstacle in msg.tracked_obstacles:
            obj_id = tracked_obstacle.obstacle.object_id
            pose = tracked_obstacle.obstacle.pose.pose
            dimensions = tracked_obstacle.obstacle
            # Create a CUBE marker for the bounding box
            box_marker = Marker()
            box_marker.header.frame_id = msg.header.frame_id
            box_marker.header.stamp = self.get_clock().now().to_msg()
            box_marker.ns = f"{self.marker_namespace}_box"
            box_marker.id = obj_id
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose = pose
            box_marker.scale.x = dimensions.width_along_x_axis
            box_marker.scale.y = dimensions.height_along_y_axis
            box_marker.scale.z = dimensions.depth_along_z_axis
            box_marker.color.r = 1.0
            box_marker.color.g = 1.0
            box_marker.color.b = 0.0
            box_marker.color.a = 0.8  # Transparency
            box_marker.lifetime = rclpy.time.Duration(seconds=0.1).to_msg()
            # Add the box marker to the array
            marker_array.markers.append(box_marker)
            # Create a TEXT_VIEW_FACING marker for the object ID
            text_marker = Marker()
            text_marker.header.frame_id = msg.header.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f"{self.marker_namespace}_id"
            text_marker.id = obj_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose
            # Position text above the box
            text_marker.pose.position.z += dimensions.depth_along_z_axis + 0.5
            text_marker.scale.z = 0.5  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = f"ID: {obj_id}"
            box_marker.lifetime = rclpy.time.Duration(seconds=3).to_msg()
            # Add the text marker to the array
            marker_array.markers.append(text_marker)
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def annotate_image_with_bounding_boxes(self):
        """Annotate the latest image with bounding boxes and IDs."""
        if self.latest_image is None or self.latest_tracked_obstacles is None:
            return  # Wait until both the image and tracked obstacles are available
        # Convert the ROS image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        for tracked_obstacle in self.latest_tracked_obstacles.tracked_obstacles:
            obj_id = tracked_obstacle.obstacle.object_id
            pose = tracked_obstacle.obstacle.pose.pose
            dimensions = tracked_obstacle.obstacle
            # self.get_logger().info(
            #     f"Obstacle ID {tracked_obstacle.obstacle.object_id}: "
            #     f"Pose ({pose.position.x}, {pose.position.y}, {pose.position.z}), "
            #     f"Dimensions (WxH: {dimensions.width_along_x_axis}x"
            #     f"{dimensions.height_along_y_axis})"
            # )
            # Assuming you have projection logic for 3D to 2D (replace this with actual projection)
            # Placeholder coordinates for now:
            top_left = (int(pose.position.x), int(pose.position.y))
            bottom_right = (
                int(pose.position.x + dimensions.width_along_x_axis),
                int(pose.position.y + dimensions.height_along_y_axis),
            )
            self.get_logger().info(
                f"Bounding box: Top-left {top_left}, Bottom-right {bottom_right}"
            )
            # Draw the bounding box
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
            # Annotate with the ID
            text = f"ID: {obj_id}"
            cv2.putText(cv_image, text, (top_left[0], top_left[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # Convert the annotated image back to a ROS image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
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
