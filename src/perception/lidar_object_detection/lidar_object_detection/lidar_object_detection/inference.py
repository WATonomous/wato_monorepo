import argparse
import glob
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import torch
import sys
sys.path.append('/home/bolty/OpenPCDet')
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from visualization_msgs.msg import Marker, MarkerArray

OPEN3D_FLAG = True


class LidarDemoNode(Node):
    def __init__(self):
        super().__init__('lidar_demo_node')
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_data', 10)
        self.bbox_publisher = self.create_publisher(MarkerArray, '/bounding_boxes', 10)

        args, cfg = self.parse_config()
        self.logger = common_utils.create_logger()
        self.logger.info('-----------------Quick Demo of OpenPCDet-------------------------')

        self.demo_dataset = DemoDataset(
            dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
            root_path=Path(args.data_path), ext=args.ext, logger=self.logger
        )

        self.model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=self.demo_dataset)
        self.model.load_params_from_file(filename=args.ckpt, logger=self.logger, to_cpu=True)
        self.model.cuda()
        self.model.eval()

        self.process_data()

    def publish_bounding_boxes(self, pred_dicts, frame_id):
        marker_array = MarkerArray()
        for idx, box in enumerate(pred_dicts[0]['pred_boxes']):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            marker.pose.orientation.w = 1.0  # Assuming no rotation; modify as needed
            marker.scale.x = float(box[3])  # Size in X direction
            marker.scale.y = float(box[4])  # Size in Y direction
            marker.scale.z = float(box[5])  # Size in Z direction
            marker.color.a = 0.8  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue
            marker_array.markers.append(marker)
        
        self.bbox_publisher.publish(marker_array)

    def process_data(self):
        with torch.no_grad():
            for idx, data_dict in enumerate(self.demo_dataset):
                data_dict = self.demo_dataset.collate_batch([data_dict])
                load_data_to_gpu(data_dict)
                pred_dicts, _ = self.model.forward(data_dict)

                # Convert and publish data
                header = self.make_header()
                points = data_dict['points'][:, :3]  # Extract the relevant columns (x, y, z)
                point_cloud_msg = self.create_cloud_xyz32(header, points)  # Assuming first 3 columns are x, y, z
                self.publisher_.publish(point_cloud_msg)
                self.publish_bounding_boxes(pred_dicts, "base_link")  # Use appropriate frame_id

        self.logger.info('Demo done.')

    def parse_config(self):
        parser = argparse.ArgumentParser(description='arg parser')
        parser.add_argument('--cfg_file', type=str, default='/home/bolty/OpenPCDet/tools/cfgs/nuscenes_models/cbgs_voxel0075_voxelnext.yaml',
                            help='specify the config for demo')
        parser.add_argument('--data_path', type=str, default='/home/bolty/data/n015-2018-11-21-19-38-26+0800__LIDAR_TOP__1542801000947820.pcd.bin',
                            help='specify the point cloud data file or directory')
        parser.add_argument('--ckpt', type=str, default="/home/bolty/OpenPCDet/models/voxelnext_nuscenes_kernel1.pth", help='specify the pretrained model')
        parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')

        args, unknown = parser.parse_known_args()
        cfg_from_yaml_file(args.cfg_file, cfg)
        return args, cfg

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        return header
    
    def create_cloud_xyz32(self, header, points):
        """
        Create a sensor_msgs/PointCloud2 message from an array of points.

        :param header: std_msgs/Header, the header of the message.
        :param points: numpy.ndarray, an Nx3 array of xyz points.
        :return: sensor_msgs/PointCloud2, the constructed PointCloud2 message.
        """
        # Create fields for the PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        y_angle = np.deg2rad(90)
        z_angle = np.deg2rad(-90)
        rotation_matrix_y = np.array([
        [np.cos(y_angle),  0, np.sin(y_angle)],
        [0,                1, 0              ],
        [-np.sin(y_angle), 0, np.cos(y_angle)]
        ])
        rotation_matrix_z = np.array([
        [np.cos(z_angle), -np.sin(z_angle), 0],
        [np.sin(z_angle),  np.cos(z_angle), 0],
        [0,                0,               1]
        ])
        # Apply transformation
        points_np = points.cpu().numpy()
        points_np_y = np.dot(points_np, rotation_matrix_y)
        points_transformed = np.dot(points_np_y, rotation_matrix_z.T)
        # Create a PointCloud2 message
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1  # Unstructured point cloud
        cloud.width = points_transformed.shape[0]
        cloud.fields = fields
        cloud.is_bigendian = False  # Assuming little endian
        cloud.point_step = 12  # FLOAT32 (4 bytes) * 3 (x, y, z)
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = bool(np.isfinite(points_transformed).all())
        cloud.data = np.asarray(points_transformed, np.float32).tobytes()

        return cloud



class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, ext='.bin'):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.root_path = root_path
        self.ext = ext
        data_file_list = glob.glob(str(root_path / f'*{self.ext}')) if self.root_path.is_dir() else [self.root_path]

        data_file_list.sort()
        self.sample_file_list = data_file_list

    def __len__(self):
        return len(self.sample_file_list)

    def __getitem__(self, index):
        if self.ext == '.bin':
            points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 5)
        elif self.ext == '.npy':
            points = np.load(self.sample_file_list[index])
        else:
            raise NotImplementedError

        input_dict = {
            'points': points,
            'frame_id': index,
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict


def main(args=None):
    rclpy.init(args=args)
    node = LidarDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()