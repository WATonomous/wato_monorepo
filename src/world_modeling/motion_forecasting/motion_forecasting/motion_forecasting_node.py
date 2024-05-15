import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist

class MotionForecastingNode(Node):
    def __init__(self):
        super().__init__('motion_forecasting_node')
        self.get_logger().info("Creating motion forecasting node...")

        # self.odom = self.create_subscription(
        #     Odometry,
        #     'odom',
        #     self.listener_callback,
        #     10)
        # self.imu = self.create_subscription(
        #     Imu,
        #     'imu',
        #     self.listener_callback2,
        #     10)
        self.map = self.create_subscription(
            OccupancyGrid,  # Replace with the actual message type to subscribe to
            'map',
            self.listener_callback3,
            10)
        self.publisher = self.create_publisher(
            String,  # Replace with the actual message type to publish
            'motion_forecasting',
            10)

    def listener_callback(self, msg):
         # Parse the x, y, and z values from the received message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        speed = abs(msg.twist.twist.linear.x)
        yaw_rate = msg.twist.twist.angular.z

        # Log the received position, velocity, and yaw rate
        self.get_logger().info(f'Received position: x={x}, y={y}, z={z}')
        self.get_logger().info(f'Received velocity: {speed}, Received yaw rate: {yaw_rate}')
        
        # Placeholder for your processing function
        future_x, future_y, future_z = self.pgp(x, y, z, speed, yaw_rate)
        
        # Prepare the message for publishing
        forecasted_msg = String()  # Replace with message type
        forecasted_msg.data = f"Future positions: x={future_x}, y={future_y}, z={future_z}"  # Assign the processed data
        self.publisher.publish(forecasted_msg)
        self.get_logger().info('Publishing: "%s"' % forecasted_msg.data)
    
    def listener_callback2(self, msg):
        linear_acceleration = msg.linear_acceleration.x

        linear_acceleration_msg = String()
        linear_acceleration_msg.data = f"Linear acceleration: {linear_acceleration}"
        self.get_logger().info('Publishing: "%s"' % linear_acceleration_msg.data)

    def listener_callback3(self, msg):
        # Access the occupancy grid data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution  # meters per cell
        map_data = msg.data  # List of occupancy data [-1, 100], where -1 is unknown

        # Log the map information
        self.get_logger().info(f'Received map: width={width}, height={height}, resolution={resolution}m/cell')

        # Further processing can go here

    def pgp(self, x, y, z, speed, yaw_rate): # Handles vehicle motion forecasting
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
