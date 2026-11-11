"""An odometry stamped constant command for plumbing checks. Not the WATO planner."""
def main():
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = Node("simforge_diagnostic_driver")
    node.declare_parameter("speed_mps", 1.0)
    node.declare_parameter("steering_rad", 0.0)
    pub = node.create_publisher(AckermannDriveStamped, "/simforge/command", 10)
    def odometry(msg):
        cmd = AckermannDriveStamped()
        cmd.header = msg.header
        cmd.drive.speed = float(node.get_parameter("speed_mps").value)
        cmd.drive.steering_angle = float(node.get_parameter("steering_rad").value)
        pub.publish(cmd)
    node.create_subscription(Odometry, "/ego/odom", odometry, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
