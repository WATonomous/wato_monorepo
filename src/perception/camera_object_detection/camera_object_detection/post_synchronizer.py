import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from vision_msgs.msg import Detection2DArray, Detection2D
from std_msgs.msg import Header
 
class CameraSyncNode(Node):
    def __init__(self):
        super().__init__('camera_sync_node')
 
        self.camera1_sub = Subscriber(self, Detection2DArray, '/camera/left/camera_detections')
        self.camera2_sub = Subscriber(self, Detection2DArray, '/camera/center/camera_detections')
        self.camera3_sub = Subscriber(self, Detection2DArray, '/camera/right/camera_detections')
 
        self.ts = ApproximateTimeSynchronizer(
            [self.camera1_sub, self.camera2_sub, self.camera3_sub],
            queue_size=10,
            slop=0.1)
 
        self.ts.registerCallback(self.callback)
 
        self.combined_detection_publisher = self.create_publisher(Detection2DArray, '/combined_detections', 10)
 
    def callback(self, camera1_msg, camera2_msg, camera3_msg):
        combined_detections = Detection2DArray()
        combined_detections.header = Header()
        combined_detections.header.stamp = self.get_clock().now().to_msg()
        combined_detections.header.frame_id = camera1_msg.header.frame_id
 
        for detection in camera1_msg.detections:
            combined_detections.detections.append(detection)
            
        for detection in camera2_msg.detections:
            combined_detections.detections.append(detection)
 
        for detection in camera3_msg.detections:
            combined_detections.detections.append(detection)
 
        self.combined_detection_publisher.publish(combined_detections)
 
def main(args=None):
    rclpy.init(args=args)
    node = CameraSyncNode()
 
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()