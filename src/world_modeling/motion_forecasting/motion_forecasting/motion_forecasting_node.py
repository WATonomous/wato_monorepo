import rclpy
from rclpy.node import Node
from sample_msgs.msg import Unfiltered
from std_msgs.msg import String

class MotionForecastingNode(Node):
    def __init__(self):
        super().__init__('motion_forecasting_node')
        self.get_logger().info("Creating motion forecasting node...")

        self.subscription = self.create_subscription(
            Unfiltered,  # Replace with the actual message type to be expected from 'unfiltered_topic'
            'unfiltered_topic',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            String,  # Replace with the actual message type to publish
            'motion_forecasting',
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)  # Log the received message
        
         # Parse the x, y, and z values from the received message
        data_parts = msg.data.split(';')
        x = float(data_parts[0].split(':')[1])
        y = float(data_parts[1].split(':')[1])
        z = float(data_parts[2].split(':')[1])
        
        # Placeholder for your processing function
        future_x, future_y, future_z = self.pgp(x, y, z)
        
        # Prepare the message for publishing
        forecasted_msg = String()  # Replace with message type
        forecasted_msg.data = f"Future positions: x={future_x}, y={future_y}, z={future_z}"  # Assign the processed data
        self.publisher.publish(forecasted_msg)
        self.get_logger().info('Publishing: "%s"' % forecasted_msg.data)

    def pgp(self, x, y, z): # Handles vehicle motion forecasting
        future_x = x + 1
        future_y = y + 1
        future_z = z + 1
        return future_x, future_y, future_z


def main(args=None):
    rclpy.init(args=args)
    motion_forecasting_node = MotionForecastingNode()
    rclpy.spin(motion_forecasting_node)
    motion_forecasting_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
