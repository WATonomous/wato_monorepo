import argparse
import glob
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import numpy as np
import torch
import sys
sys.path.append('/home/bolty/OpenPCDet')
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
import sensor_msgs.point_cloud2 as pc2

from visual_utils import open3d_vis_utils as V
OPEN3D_FLAG = True


class LidarDemoNode(Node):
    def __init__(self):
        super().__init__('lidar_demo_node')
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_data', 10)

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

    def process_data(self):
        with torch.no_grad():
            for idx, data_dict in enumerate(self.demo_dataset):
                data_dict = self.demo_dataset.collate_batch([data_dict])
                load_data_to_gpu(data_dict)
                pred_dicts, _ = self.model.forward(data_dict)

                # Convert and publish data
                header = self.make_header()
                point_cloud_msg = pc2.create_cloud_xyz32(header, data_dict['points'][:, 1:4])  # Assuming first 3 columns are x, y, z
                self.publisher_.publish(point_cloud_msg)

        self.logger.info('Demo done.')

    def parse_config(self):
        parser = argparse.ArgumentParser(description='arg parser')
        parser.add_argument('--cfg_file', type=str, default='cfgs/nuscenes_models/cbgs_second_multihead.yaml',
                            help='specify the config for demo')
        parser.add_argument('--data_path', type=str, default='demo_data',
                            help='specify the point cloud data file or directory')
        parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')
        parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')

        args, unknown = parser.parse_known_args()
        cfg_from_yaml_file(args.cfg_file, cfg)
        return args, cfg

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'lidar_frame'
        return header


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