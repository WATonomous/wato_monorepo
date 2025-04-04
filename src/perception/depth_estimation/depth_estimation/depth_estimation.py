import os
import time
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
        self.declare_parameter("small_model_path", "/perception_models/depth_anything_v2_small.pth")
        self.declare_parameter("large_model_path", "/perception_models/depth_anything_v2_large.pth")

        self.camera_topic = self.get_parameter("camera_topic").value
        self.publish_depth_img_topic = self.get_parameter("publish_depth_img_topic").value
        self.publish_depth_pcl_topic = self.get_parameter("publish_depth_pcl_topic").value
        self.small_model = self.get_parameter("small_model_path").value
        self.large_model = self.get_parameter("large_model_path").value

        self.encoder_choices = ['vits', 'vitb', 'vitl', 'vitg']
        self.encoder = self.encoder_choices[2]
        self.max_depth = 80  # in meters, I believe
        self.focal_length_x = 470.4
        self.focal_length_y = 470.4
        self.outdir = "/runs/"
        self.img_id = 0

        # Determine the device to use (CUDA, MPS, or CPU)
        if torch.cuda.is_available():
            self.DEVICE = 'cuda'
        elif torch.backends.mps.is_available():
            self.DEVICE = 'mps'
        else:
            self.DEVICE = 'cpu'
        self.get_logger().info(f"Using device: {self.DEVICE}")
        self.get_logger().info(f"CUDA available: {torch.cuda.is_available()}")
        self.get_logger().info(f"MPS available: {torch.backends.mps.is_available()}")

        # Model configuration based on the chosen encoder
        self.model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        # Initialize the DepthAnythingV2 model with the specified configuration
        self.depth_anything = DepthAnythingV2(**self.model_configs[self.encoder])
        # Default to using the small model
        self.depth_anything.load_state_dict(torch.load(self.small_model, map_location='cpu'))
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

    def create_cloud(self, header, fields, points):
        """
        Create a sensor_msgs/PointCloud2 message.
        """
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        # Unorganized point cloud: height is 1, width is the number of points.
        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        # Each point has 3 float32 values, so each point is 12 bytes.
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * points.shape[0]
        cloud_msg.is_dense = True
        # Convert the points array to a bytes object.
        cloud_msg.data = points.astype(np.float32).tobytes()
        return cloud_msg

    def image_callback(self, msg):
        t_start = time.time()
        self.get_logger().debug("Received image")

        # Decode the compressed image
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return
        t_decode = time.time()
        self.get_logger().debug(f"Image decoding time: {t_decode - t_start:.4f} sec")

        # Create the output directory if it doesn't exist
        os.makedirs(self.outdir, exist_ok=True)

        # === generate depth img ===
        color_image = PILImage.fromarray(cv_image).convert('RGB')
        width, height = color_image.size

        # Inference (depth estimation)
        t_infer_start = time.time()
        pred = self.depth_anything.infer_image(cv_image, height)
        t_infer_end = time.time()
        self.get_logger().debug(f"Inference time: {t_infer_end - t_infer_start:.4f} sec")

        # Resize depth prediction to match the original image size
        resized_pred = PILImage.fromarray(pred).resize((width, height), PILImage.NEAREST)
        self.publish_depth_img(resized_pred, msg)

        # Optionally, disable disk writes during timing tests:
        cv2.imwrite(os.path.join(self.outdir, f"depth_{self.img_id}.png"), np.array(resized_pred))

        # === generate point cloud ===
        t_pcl_start = time.time()
        x, y = np.meshgrid(np.arange(width), np.arange(height))
        x = (x - width / 2) / self.focal_length_x
        y = (y - height / 2) / self.focal_length_y
        z = np.array(resized_pred)
        points = np.stack((np.multiply(x, z), np.multiply(y, z), z), axis=-1).reshape(-1, 3)
        colors = np.array(color_image).reshape(-1, 3) / 255.0

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcl_msg = self.convert_open3d_to_pointcloud2(pcd, msg)
        o3d.io.write_point_cloud(os.path.join(self.outdir, f"depth_{self.img_id}.ply"), pcd)
        t_pcl_end = time.time()
        self.get_logger().debug(f"Point cloud generation time: {t_pcl_end - t_pcl_start:.4f} sec")

        # Publish point cloud
        self.depth_pcl_publisher.publish(pcl_msg)
        self.img_id += 1

        t_end = time.time()
        self.get_logger().debug(f"Total callback time: {t_end - t_start:.4f} sec")

    def publish_depth_img(self, depth_img, msg):
        depth_img_np = np.array(depth_img)
        # Normalize depth values to the range [0, 255] for visualization.
        normalized = cv2.normalize(depth_img_np, None, 0, 255, cv2.NORM_MINMAX)
        depth_uint8 = normalized.astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)

        img_msg = self.cv_bridge.cv2_to_imgmsg(depth_color, "bgr8")
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.depth_img_publisher.publish(img_msg)

    def convert_open3d_to_pointcloud2(self, o3d_pc, msg, frame_id="map"):
        points = np.asarray(o3d_pc.points)
        header = std_msgs.msg.Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        pf_x = PointField()
        pf_x.name = 'x'
        pf_x.offset = 0
        pf_x.datatype = PointField.FLOAT32
        pf_x.count = 1

        pf_y = PointField()
        pf_y.name = 'y'
        pf_y.offset = 4
        pf_y.datatype = PointField.FLOAT32
        pf_y.count = 1

        pf_z = PointField()
        pf_z.name = 'z'
        pf_z.offset = 8
        pf_z.datatype = PointField.FLOAT32
        pf_z.count = 1

        fields = [pf_x, pf_y, pf_z]
        point_cloud_msg = self.create_cloud(header, fields, points)
        return point_cloud_msg


def main(args=None):
    rclpy.init(args=args)
    depth_anything = DepthAnything()
    rclpy.spin(depth_anything)
    depth_anything.destroy_node()
    rclpy.shutdown()
