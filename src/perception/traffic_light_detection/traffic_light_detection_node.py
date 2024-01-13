import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class TrafficLightDetectionNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detection_node')
        self.yolo_model = YOLO("traffic_light.pt")
        self.cv_bridge = CvBridge()

        self.publisher_ = self.create_publisher(Image, 'processed_image_topic', 10)
        self.subscriber_ = self.create_subscription(Image, 'input_image_topic', self.image_callback, 10)

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        # Predict using YOLO model
        results = self.yolo_model.predict(cv_image)

        # Process results (draw boxes, labels, etc.)
        for box in results[0].boxes:
            class_name = results[0].names[box.cls[0].item()]
            confidence = box.conf.item()  # Convert tensor to a Python number
            x_start, y_start, x_end, y_end = map(int, box.xyxy[0].tolist())

            cv2.rectangle(cv_image, (x_start, y_start), (x_end, y_end), (128, 0, 128), 2)
            cv2.putText(cv_image, f"{class_name}: {confidence:.2f}", (x_start, y_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 0, 128), 2)

        # Convert back to ROS Image message and publish
        output_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.publisher_.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
