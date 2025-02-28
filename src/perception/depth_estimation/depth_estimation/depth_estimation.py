import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import open3d as o3d
import std_msgs.msg
import rclpy
from PIL import Image as PILImage


from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from rclpy.node import Node

from depth_anything_v2.dpt import DepthAnythingV2

class DepthAnything(Node):

    def __init__(self):

        super().__init__("depth_anything_node")

        self.declare_parameter("camera_topic", "/CAM_FRONT/image_rect_compressed")
        self.declare_parameter("publish_depth_img_topic", "/camera/right/depth_img")
        self.declare_parameter("publish_depth_pcl_topic", "/camera/right/depth_plc")
        self.declare_parameter("model_path", "/home/bolty/ament_ws/src/depth_anything_v2_large.pth")

        self.camera_topic = self.get_parameter("camera_topic").value
        self.publish_depth_img_topic = self.get_parameter("publish_depth_img_topic").value
        self.publish_depth_pcl_topic = self.get_parameter("publish_depth_pcl_topic").value
        self.model =  self.get_parameter("model_path").value

        self.encoder_choices = ['vits', 'vitb', 'vitl', 'vitg']
        self.encoder = self.encoder_choices[2]
        self.max_depth = 80 # in meters i believe
        self.focal_length_x = 470.4
        self.focal_length_y = 470.4
        self.outdir = "/runs/"
        self.img_id = 0
        # Determine the device to use (CUDA, MPS, or CPU)
        self.DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        # Model configuration based on the chosen encoder
        self.model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        # Initialize the DepthAnythingV2 model with the specified configuration
        self.depth_anything = DepthAnythingV2(**self.model_configs[self.encoder])
        self.depth_anything.load_state_dict(torch.load(self.model, map_location='cpu'))
        self.depth_anything = self.depth_anything.to(self.DEVICE).eval()

        self.cv_bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            10
        )

        self.depth_img_publisher = self.create_publisher(
            Image,
            self.publish_depth_img_topic,
            10
        )
        self.depth_pcl_publisher = self.create_publisher(
            PointCloud2,
            self.publish_depth_pcl_topic,
            10
        )

    def image_callback(self, msg):
        self.get_logger().debug("Received image")

        image = msg # is already of type Image
        cv_image = None

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return
        
        # Create the output directory if it doesn't exists
        os.makedirs(self.outdir, exist_ok=True)
        
        # === generate depth img ===

        # Load the image
        color_image = PILImage.fromarray(cv_image).convert('RGB')
        width, height = color_image.size

        # Read the image using OpenCV
        # image = cv2.imread(image)
        pred = self.depth_anything.infer_image(cv_image, height)

        # Resize depth prediction to match the original image size
        resized_pred = PILImage.fromarray(pred).resize((width, height), PILImage.NEAREST)
        self.publish_depth_img(resized_pred, msg)
        cv2.imwrite(os.path.join(self.outdir, f"depth_{self.img_id}.png"), np.array(resized_pred))

        # === generate pcl ===

        # Generate mesh grid and calculate point cloud coordinates
        x, y = np.meshgrid(np.arange(width), np.arange(height))
        x = (x - width / 2) / self.focal_length_x
        y = (y - height / 2) / self.focal_length_y
        z = np.array(resized_pred)
        points = np.stack((np.multiply(x, z), np.multiply(y, z), z), axis=-1).reshape(-1, 3)
        colors = np.array(color_image).reshape(-1, 3) / 255.0

        # Create the point cloud and save it to the output directory
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcl_msg = self.convert_open3d_to_pointcloud2(pcd, msg)
        o3d.io.write_point_cloud(os.path.join(self.outdir,  f"depth_{self.img_id}.ply"), pcd)

        # publish
        
        self.depth_pcl_publisher.publish(pcl_msg)
        self.img_id += 1

    def publish_depth_img(self, depth_img, msg):
        depth_img_np = np.array(depth_img)
        img_msg = self.cv_bridge.cv2_to_imgmsg(depth_img_np, "bgr8")
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.depth_img_publisher.publish(img_msg)

    def convert_open3d_to_pointcloud2(self, o3d_pc, msg, frame_id="map"):
        points = np.asarray(o3d_pc.points)
        header = std_msgs.msg.Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        point_cloud_msg = PointCloud2.create_cloud(header, fields, points)
        return point_cloud_msg

def main(args=None):
    rclpy.init(args=args)

    depth_anything = DepthAnything()

    rclpy.spin(depth_anything)

    depth_anything.destroy_node()
    rclpy.shutdown()