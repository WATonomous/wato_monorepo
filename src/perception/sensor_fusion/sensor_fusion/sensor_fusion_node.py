import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from message_filters import Subscriber, TimeSynchronizer
from cv_bridge import CvBridge
import json

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera topics
        self.subscribers = [
            Subscriber(self, Image, f'/camera{i}/image_raw') for i in range(1, 9)
        ]
        
        # TimeSynchronizer to synchronize messages
        self.ts = TimeSynchronizer(self.subscribers, 10)
        self.ts.registerCallback(self.image_callback)
        
        # Publisher for synchronized images
        self.publisher = self.create_publisher(String, 'synchronized_images', 10)
    
    def image_callback(self, *images):
        # Convert ROS images to OpenCV format and encode them
        encoded_images = [self.bridge.cv2_to_imgmsg(cv2.imencode('.jpg', self.bridge.imgmsg_to_cv2(img, 'bgr8'))[1], encoding='bgr8') for img in images]
        
        # Create a JSON object to hold the images
        images_dict = {f'camera{i+1}': img.data.tobytes().hex() for i, img in enumerate(encoded_images)}
        
        # Publish the synchronized images as an array of images
        self.publisher.publish(json.dumps(images_dict))

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
