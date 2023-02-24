import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
# from deepracer_interfaces_pkg.msg import CameraMsg

from yolov5.models.common import DetectMultiBackend
from yolov5.utils.augmentations import letterbox
from yolov5.utils.general import check_img_size, scale_segments, non_max_suppression


import os
from sensor_msgs.msg import Image

from easydict import EasyDict

# https://github.com/ultralytics/yolov5/issues/5304


import cv2
from cv_bridge import CvBridge, CvBridgeError

import torch

CAMERA_TOPIC='/camera_pkg/video_mjpeg'
MODEL_PATH="/home/leungjch/Downloads/yolov5s.pt"

class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_param_node')


        print(os.getcwd())

        self.model_path = self.declare_parameter("model_path", MODEL_PATH).value
        self.image_size = self.declare_parameter("image_size", 480).value
        self.half = False
        self.augment = False
        # self.config = EasyDict(config)

        print("Creating sub")
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        print("Done creating sub")

        # set device
        self.device = torch.device("cpu")

        # CV bridge
        self.cv_bridge = CvBridge()

        # load YOLOv5 model
        self.model = DetectMultiBackend(
            self.model_path, device=self.device, dnn=False, fp16=False
        )

        self.names = (
            self.model.module.names
            if hasattr(self.model, "module")
            else self.model.names
        )


        self.stride = int(self.model.stride)


    def preprocess_image(self, cv_image):
        """
        Preprocess the image by resizing, padding and rearranging the dimensions
        Parameters: 
            cv_image: A numpy or cv2 image of shape (w,h,3)
        Returns:
            torch.Tensor image for model input of shape (1,3,w,h)
        """
        # Padded resize
        img = cv_image
        img = letterbox(cv_image, self.image_size, stride=self.stride)[0]

        # Convert
        img = img.transpose(2, 0, 1)

        # further conversion
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        img = img.unsqueeze(0)

        return img


    def listener_callback(self, msg):
        print("Got listener")
        # msg.images is a list of images (size 1 for mono, size 2 for stereo)
        self.get_logger().info('I heard: "%s"' % len(msg.images))

        for image in msg.images:
            # convert ros Image to cv::Mat
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            except CvBridgeError as e:
                rospy.ERROR(e)
                return

            # preprocess image and run through prediction
            img = self.preprocess_image(cv_image)
            pred = self.model(img)

            print(pred)
            pred = non_max_suppression(pred) #nms function used same as yolov5 detect.py


            
            for i, det in enumerate(pred):  # per image

                print(det.shape)
                if len(det):
                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        print(cls)
                        c = int(cls)  # integer class
                        label = self.names[int(cls)] 
                        print(label)
                # Save results (image with detections)
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()