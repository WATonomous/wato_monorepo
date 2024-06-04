import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from std_msgs.msg import Header

from ultralytics.utils.plotting import Annotator, colors

from cv_bridge import CvBridgeError
 
class CameraSyncNode(Node): # synchronizes visualizations
    def __init__(self):
        super().__init__('camera_sync_node')

        self.camera_img_sub = Subscriber(self, Image , '/camera/right/image_color')
 
        self.camera1_sub = Subscriber(self, Detection2DArray, '/camera/left/camera_detections')
        self.camera2_sub = Subscriber(self, Detection2DArray, '/camera/center/camera_detections')
        self.camera3_sub = Subscriber(self, Detection2DArray, '/camera/right/camera_detections')
 
        self.ts = ApproximateTimeSynchronizer(
            [self.camera_img_sub, self.camera1_sub, self.camera2_sub, self.camera3_sub],
            queue_size=10,
            slop=0.1)
 
        self.ts.registerCallback(self.callback)
 
        self.combined_detection_publisher = self.create_publisher(Detection2DArray, '/combined_detections', 10)
        self.vis_publisher = self.create_publisher(Image, '/annotated_img')

    def process_img(self, image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return
        return cv_image
    
    def postprocess_detections(self, detections, annotator):
        """
        Post-process draws bouningboxes on camera image.

        Parameters:
            detections: A list of dict with the format
                {
                    "label": str,
                    "bbox": [float],
                    "conf": float
                }
            annotator: A ultralytics.yolo.utils.plotting.Annotator for the current image

        Returns:
            processed_detections: filtered detections
            annotator_img: image with bounding boxes drawn on
        """
        processed_detections = detections

        for det in detections:
            label = f'{det["label"]} {det["conf"]:.2f}'
            x1, y1, w1, h1 = det["bbox"]
            xyxy = [x1, y1, x1 + w1, y1 + h1]
            annotator.box_label(xyxy, label, color=colors(1, True))

        annotator_img = annotator.result()
        return (processed_detections, annotator_img)
    
    def publish_vis(self, annotated_img, msg):
        # Publish visualizations
        imgmsg = self.cv_bridge.cv2_to_imgmsg(annotated_img, "bgr8")
        imgmsg.header.stamp = msg.header.stamp
        imgmsg.header.frame_id = msg.header.frame_id
        self.vis_publisher.publish(imgmsg)
 
    def callback(self, camera_img_sub, camera1_msg, camera2_msg, camera3_msg):
        combined_detections = Detection2DArray()
        combined_detections.header = Header()
        combined_detections.header.stamp = self.get_clock().now().to_msg()
        combined_detections.header.frame_id = camera1_msg.header.frame_id

        cv_image = self.process_img(camera_img_sub)
 
        for detection in camera1_msg.detections:
            combined_detections.detections.append(detection)
            
        for detection in camera2_msg.detections:
            combined_detections.detections.append(detection)
 
        for detection in camera3_msg.detections:
            combined_detections.detections.append(detection)

        annotator = Annotator(
            cv_image,
            line_width=self.line_thickness,
            example=str(self.names),
        )
        (combined_detections, annotated_img) = self.postprocess_detections(combined_detections, annotator)
 
        self.combined_detection_publisher.publish(combined_detections)
        self.publish_vis(annotated_img, camera_img_sub)
 
def main(args=None):
    rclpy.init(args=args)
    node = CameraSyncNode()
 
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()