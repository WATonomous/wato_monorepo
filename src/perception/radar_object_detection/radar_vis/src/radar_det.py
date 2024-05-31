import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from radar_velocity_detection.msg import RadarDetections
import numpy as np

class RadarVelocityDetectionNode(Node):
    def __init__(self):
        super().__init__('radar_velocity_detection_node')
        self.subscription = self.create_subscription(
            RadarDetections,
            '/RADAR_TOP',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'velocity', 10)

    def listener_callback(self, msg):
        velocities = self.detect_velocities(msg.detections)
        avg_velocity = np.mean(velocities) if len(velocities) > 0 else 0.0
        self.publisher_.publish(Float32(data=avg_velocity))
        self.get_logger().info(f'Average velocity: {avg_velocity:.2f} m/s')

    def detect_velocities(self, detections):
        velocities = [np.sqrt(d.vx**2 + d.vy**2) for d in detections]
        return velocities

def main(args=None):
    rclpy.init(args=args)
    node = RadarVelocityDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
