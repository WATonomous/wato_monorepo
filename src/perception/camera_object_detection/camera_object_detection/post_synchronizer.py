import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

from ultralytics.utils.plotting import Annotator, colors

from cv_bridge import CvBridge, CvBridgeError
 
class CameraSyncNode(Node): # synchronizes visualizations
    def __init__(self):
        super().__init__('camera_sync_node')
        self.get_logger().info("Camera Sync Node")

        self.declare_parameter("camera_img_topic", "/camera/right/image_color")
        self.declare_parameter("camera_detection_topic", "/camera/right/camera_detections")
        self.declare_parameter("traffic_signs_topic", "/traffic_signs/right")
        self.declare_parameter("traffic_lights_topic", "/traffic_lights/right")
        self.declare_parameter("combined_detection_publisher", "/camera/right/combined_detections")
        self.declare_parameter("publish_vis_topic", "/camera/right/annotated_img")

        self.camera_img_topic = self.get_parameter("camera_img_topic").value
        self.camera_detection_topic = self.get_parameter("camera_detection_topic").value
        self.traffic_signs_topic = self.get_parameter("traffic_signs_topic").value
        self.traffic_lights_topic = self.get_parameter("traffic_lights_topic").value
        self.combined_detection_topic = self.get_parameter("combined_detection_publisher").value
        self.publish_vis_topic = self.get_parameter("publish_vis_topic").value

        self.camera_img_sub = Subscriber(self, Image, self.camera_img_topic)
        self.camera_detection_sub = Subscriber(self, Detection2DArray, self.camera_detection_topic)
        self.traffic_signs_sub = Subscriber(self, Detection2DArray, self.traffic_signs_topic)
        self.traffic_lights_sub = Subscriber(self, Detection2DArray, self.traffic_lights_topic)

        self.cv_bridge = CvBridge()
        self.line_thickness = 1
        self.names = []
 
        self.ts = ApproximateTimeSynchronizer(
            [self.camera_img_sub,
             self.camera_detection_sub,
             self.traffic_signs_sub,
             self.traffic_lights_sub],
            queue_size=10,
            slop=0.1)
 
        self.ts.registerCallback(self.callback)
 
        self.combined_detection_publisher = self.create_publisher(Detection2DArray, self.combined_detection_topic, 10)
        self.vis_publisher = self.create_publisher(Image, self.publish_vis_topic, 10)

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
    
    def publish_detections(self, detections, msg, feed):
        # Publish detections to an detectionList message
        detection2darray = Detection2DArray()

        # fill header for detection list
        detection2darray.header.stamp = msg.header.stamp
        detection2darray.header.frame_id = msg.header.frame_id
        # populate detection list
        if detections is not None and len(detections):
            for detection in detections:
                detection2d = Detection2D()
                detection2d.header.stamp = self.get_clock().now().to_msg()
                detection2d.header.frame_id = msg.header.frame_id
                detected_object = ObjectHypothesisWithPose()
                detected_object.hypothesis.class_id = detection["label"]
                detected_object.hypothesis.score = detection["conf"]
                detection2d.results.append(detected_object)
                detection2d.bbox.center.position.x = detection["bbox"][0]
                detection2d.bbox.center.position.y = detection["bbox"][1]
                detection2d.bbox.size_x = detection["bbox"][2]
                detection2d.bbox.size_y = detection["bbox"][3]

                # append detection to detection list
                detection2darray.detections.append(detection2d)

        self.combined_detection_publisher.publish(detection2darray)
    
    def publish_vis(self, annotated_img, msg):
        # Publish visualizations
        imgmsg = self.cv_bridge.cv2_to_imgmsg(annotated_img, "bgr8")
        imgmsg.header.stamp = msg.header.stamp
        imgmsg.header.frame_id = msg.header.frame_id
        self.get_logger().info("Publish img")
        self.vis_publisher.publish(imgmsg)
 
    def callback(
        self,
        camera_img_sub,
        camera_detection_sub,
        traffic_signs_sub,
        traffic_lights_sub
    ):
        cv_image = self.process_img(camera_img_sub)

        combined_detections = camera_detection_sub.detections + traffic_lights_sub.detections + traffic_signs_sub.detections

        annotator = Annotator(
            cv_image,
            line_width=self.line_thickness,
            example=str(self.names),
        )
        
        detections = []
        for det in combined_detections:
            obj_with_pose = det.results[0]
            detections.append(
                {
                    "label": obj_with_pose.hypothesis.class_id,
                    "conf": obj_with_pose.hypothesis.score,
                    "bbox": [
                        det.bbox.center.position.x,
                        det.bbox.center.position.y,
                        det.bbox.size_x,
                        det.bbox.size_y
                    ],
                }
            )
        (combined_detections, annotated_img) = self.postprocess_detections(detections, annotator)

        self.publish_detections(combined_detections, camera_detection_sub, "")
        self.publish_vis(annotated_img, camera_img_sub)
 
def main(args=None):
    rclpy.init(args=args)
    node = CameraSyncNode()
 
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()