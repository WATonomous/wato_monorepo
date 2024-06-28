import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
import tf_transformations
from rclpy.time import Time

class MarkerVelocityEstimator(Node):
    def __init__(self):
        super().__init__('marker_velocity_estimator')

        # Create a TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the marker topic
        self.marker_sub = self.create_subscription(Marker, '/visualization_marker', self.marker_callback, 10)

        self.previous_marker = None

    def marker_callback(self, marker):
        if self.previous_marker is None:
            self.previous_marker = marker
            return

        try:
            # Get current and previous marker transforms
            trans_now = self.tf_buffer.lookup_transform('world', marker.header.frame_id, Time())
            trans_prev = self.tf_buffer.lookup_transform('world', self.previous_marker.header.frame_id, self.previous_marker.header.stamp)
        except Exception as ex:
            self.get_logger().warn(f'TF Exception: {ex}')
            return

        # Compute the difference in positions
        delta_position = [
            trans_now.transform.translation.x - trans_prev.transform.translation.x,
            trans_now.transform.translation.y - trans_prev.transform.translation.y,
            trans_now.transform.translation.z - trans_prev.transform.translation.z
        ]

        # Compute the difference in time
        delta_time = (marker.header.stamp.sec + marker.header.stamp.nanosec * 1e-9) - (self.previous_marker.header.stamp.sec + self.previous_marker.header.stamp.nanosec * 1e-9)

        if delta_time == 0:
            self.get_logger().warn("Delta time is zero, skipping velocity calculation.")
            return

        # Compute velocity
        velocity = Twist()
        velocity.linear.x = delta_position[0] / delta_time
        velocity.linear.y = delta_position[1] / delta_time
        velocity.linear.z = delta_position[2] / delta_time

        # Log the velocity
        self.get_logger().info(f"Marker velocity: [x: {velocity.linear.x}, y: {velocity.linear.y}, z: {velocity.linear.z}]")

        # Update previous marker
        self.previous_marker = marker

def main(args=None):
    rclpy.init(args=args)

    estimator = MarkerVelocityEstimator()

    rclpy.spin(estimator)

    # Clean up and shutdown
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
