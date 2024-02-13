#! /usr/bin/env python3.8
"""
Created on Thu Aug  6 11:27:43 2020

@author: Javier del Egido Sierra and Carlos Gómez-Huélamo

===

Modified on 23 Dec 2022
@author: Kin ZHANG (https://kin-zhang.github.io/)

Part of codes also refers: https://github.com/kwea123/ROS_notes
"""

# General use imports
import os
import time
import glob
from pathlib import Path

# ROS imports
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker

# Math and geometry imports
import math
import numpy as np
import torch

# OpenPCDet imports
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
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = points_sum.shape[0]
    msg.is_dense = int(np.isfinite(points_sum).all())
    msg.data = np.asarray(points_sum, np.float32).tostring()
    return msg

def rslidar_callback(msg):
    select_boxs, select_types = [],[]
    if proc_1.no_frame_id:
        proc_1.set_viz_frame_id(msg.header.frame_id)
        print(f"{bc.OKGREEN} setting marker frame id to lidar: {msg.header.frame_id} {bc.ENDC}")
        proc_1.no_frame_id = False

    frame = msg.header.seq # frame id -> not timestamp
    msg_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    np_p = get_xyz_points(msg_cloud, True)
    scores, dt_box_lidar, types, pred_dict = proc_1.run(np_p, frame)
    for i, score in enumerate(scores):
        if score>threshold:
            select_boxs.append(dt_box_lidar[i])
            select_types.append(pred_dict['name'][i])
    if(len(select_boxs)>0):
        proc_1.pub_rviz.publish_3dbox(np.array(select_boxs), -1, pred_dict['name'])
        print_str = f"Frame id: {frame}. Prediction results: \n"
        for i in range(len(pred_dict['name'])):
            print_str += f"Type: {pred_dict['name'][i]:.3s} Prob: {scores[i]:.2f}\n"
        print(print_str)
    else:
        print(f"\n{bc.FAIL} No confident prediction in this time stamp {bc.ENDC}\n")
    print(f" -------------------------------------------------------------- ")

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
            points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 4)
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

class Processor_ROS:
    def __init__(self, config_path, model_path):
        self.points = None
        self.config_path = config_path
        self.model_path = model_path
        self.device = None
        self.net = None
        self.voxel_generator = None
        self.inputs = None
        self.pub_rviz = None
        self.no_frame_id = True
        self.rate = RATE_VIZ

    def set_pub_rviz(self, box3d_pub, marker_frame_id = 'velodyne'):
        self.pub_rviz = Draw3DBox(box3d_pub, marker_frame_id, self.rate)
    
    def set_viz_frame_id(self, marker_frame_id):
        self.pub_rviz.set_frame_id(marker_frame_id)

    def initialize(self):
        self.read_config()
        
    def read_config(self):
        config_path = self.config_path
        cfg_from_yaml_file(self.config_path, cfg)
        self.logger = common_utils.create_logger()
        self.demo_dataset = DemoDataset(
            dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
            root_path=Path("/home/kin/workspace/OpenPCDet/tools/000002.bin"),
            ext='.bin')
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.net = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=self.demo_dataset)
        print("Model path: ", self.model_path)
        self.net.load_params_from_file(filename=self.model_path, logger=self.logger, to_cpu=True)
        self.net = self.net.to(self.device).eval()

    def get_template_prediction(self, num_samples):
        ret_dict = {
            'name': np.zeros(num_samples), 'truncated': np.zeros(num_samples),
            'occluded': np.zeros(num_samples), 'alpha': np.zeros(num_samples),
            'bbox': np.zeros([num_samples, 4]), 'dimensions': np.zeros([num_samples, 3]),
            'location': np.zeros([num_samples, 3]), 'rotation_y': np.zeros(num_samples),
            'score': np.zeros(num_samples), 'boxes_lidar': np.zeros([num_samples, 7])
        }
        return ret_dict

    def run(self, points, frame):
        t_t = time.time()
        num_features = 4 # X,Y,Z,intensity       
        self.points = points.reshape([-1, num_features])

        timestamps = np.empty((len(self.points),1))
        timestamps[:] = frame

        # self.points = np.append(self.points, timestamps, axis=1)
        self.points[:,0] += move_lidar_center

        input_dict = {
            'points': self.points,
            'frame_id': frame,
        }

        data_dict = self.demo_dataset.prepare_data(data_dict=input_dict)
        data_dict = self.demo_dataset.collate_batch([data_dict])
        load_data_to_gpu(data_dict)

        torch.cuda.synchronize()
        t = time.time()

        pred_dicts, _ = self.net.forward(data_dict)
        
        torch.cuda.synchronize()
        inference_time = time.time() - t
        inference_time_list.append(inference_time)

        boxes_lidar = pred_dicts[0]["pred_boxes"].detach().cpu().numpy()
        scores = pred_dicts[0]["pred_scores"].detach().cpu().numpy()
        types = pred_dicts[0]["pred_labels"].detach().cpu().numpy()

        pred_boxes = np.copy(boxes_lidar)
        pred_dict = self.get_template_prediction(scores.shape[0])

        pred_dict['name'] = np.array(cfg.CLASS_NAMES)[types - 1]
        pred_dict['score'] = scores
        pred_dict['boxes_lidar'] = pred_boxes

        return scores, boxes_lidar, types, pred_dict
 
if __name__ == "__main__":
    no_frame_id = False
    proc_1 = Processor_ROS(cfg_root, model_path)
    print(f"\n{bc.OKCYAN}Config path: {bc.BOLD}{cfg_root}{bc.ENDC}")
    print(f"{bc.OKCYAN}Model path: {bc.BOLD}{model_path}{bc.ENDC}")

    proc_1.initialize()
    rospy.init_node('inference')
    sub_lidar_topic = [pointcloud_topic]

    cfg_from_yaml_file(cfg_root, cfg)
    
    sub_ = rospy.Subscriber(sub_lidar_topic[0], PointCloud2, rslidar_callback, queue_size=1, buff_size=2**24)
    pub_rviz = rospy.Publisher('detect_3dbox',MarkerArray, queue_size=10)
    proc_1.set_pub_rviz(pub_rviz)
    print(f"{bc.HEADER} ====================== {bc.ENDC}")
    print(" ===> [+] PCDet ros_node has started. Try to Run the rosbag file")
    print(f"{bc.HEADER} ====================== {bc.ENDC}")

    rospy.spin()