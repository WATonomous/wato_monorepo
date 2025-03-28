# pylint: disable=wrong-import-position
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import ObjectHypothesisWithPose, Detection3D, Detection3DArray, VisionInfo
from visualization_msgs.msg import Marker, MarkerArray
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
import torch
import numpy as np
from rclpy.node import Node
import rclpy
import argparse
import sys
sys.path.append("/home/bolty/OpenPCDet")


class LidarObjectDetection(Node):
    def __init__(self):
        super().__init__('lidar_object_detection')
        self.declare_parameter("model_path")
        self.declare_parameter("model_config_path")
        self.declare_parameter("lidar_topic")
        self.model_path = self.get_parameter("model_path").value
        self.model_config_path = self.get_parameter("model_config_path").value
        self.lidar_data = self.get_parameter("lidar_topic").value
        self.publish_detection = self.get_parameter(
            'enable_detection').get_parameter_value().bool_value

        self.label_mapping = {}
        self.subscription = self.create_subscription(
            VisionInfo,
            'vision_info',
            self.vision_info_callback,
            10)
        self.viz_publisher = self.create_publisher(MarkerArray, "/lidar_detections_viz", 10)
        self.detections_publisher = self.create_publisher(Detection3DArray, "/lidar_detections", 10)

        self.subscription = self.create_subscription(
            PointCloud2, self.lidar_data, self.point_cloud_callback, 10
        )

        args, cfg = self.parse_config()
        self.logger = common_utils.create_logger()

        self.lidar_dataloader = LidarDatalodaer(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
            logger=self.logger,
        )

        self.model = build_network(
            model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=self.lidar_dataloader
        )
        self.model.load_params_from_file(filename=args.ckpt, logger=self.logger, to_cpu=True)
        self.model.cuda()
        self.model.eval()

    def vision_info_callback(self, msg):
        self.label_mapping = msg.class_map

    def point_cloud_callback(self, msg):
        points = self.pointcloud2_to_xyz_array(msg)
        data_dict = {
            "points": points,
            "frame_id": msg.header.frame_id,
        }
        data_dict = self.lidar_dataloader.prepare_data(data_dict=data_dict)
        data_dict = self.lidar_dataloader.collate_batch([data_dict])
        load_data_to_gpu(data_dict)

        with torch.no_grad():
            pred_dicts, _ = self.model.forward(data_dict)

        original_timestamp = msg.header.stamp
        self.publish_bounding_boxes(msg, pred_dicts, original_timestamp)

    def pointcloud2_to_xyz_array(self, cloud_msg):
        num_points = cloud_msg.width * cloud_msg.height
        cloud_array = np.frombuffer(cloud_msg.data, dtype=np.float32)
        num_fields = cloud_msg.point_step // 4
        cloud_array = cloud_array.reshape(num_points, num_fields)
        if cloud_array.shape[1] > 4:
            cloud_array = cloud_array[:, :4]
        if cloud_array.shape[1] <= 4:
            timestamp = np.full((num_points, 1), fill_value=0.0, dtype=np.float32)
            cloud_array = np.hstack((cloud_array, timestamp))
        return cloud_array

    def publish_bounding_boxes(self, pointcloud_msg, pred_dicts, original_timestamp):
        marker_array = MarkerArray()
        detections = Detection3DArray()
        detections.header = pointcloud_msg.header
        for idx, (box, score) in enumerate(zip(pred_dicts[0]["pred_boxes"], pred_dicts[0]["pred_scores"])):
            if score > 0.6:
                marker = Marker()
                marker.header = pointcloud_msg.header
                marker.header.stamp = original_timestamp
                marker.id = idx
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(box[0])
                marker.pose.position.y = float(box[1])
                marker.pose.position.z = float(box[2])

                # Calculate orientation quaternion
                yaw = float(box[6]) + 1e-10
                marker.pose.orientation.z = np.sin(yaw / 2)
                marker.pose.orientation.w = np.cos(yaw / 2)

                marker.scale.x = float(box[3])
                marker.scale.y = float(box[4])
                marker.scale.z = float(box[5])
                marker.color.a = 0.8
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = float(pred_dicts[0]["pred_labels"][idx]) / 3

                class_id = pred_dicts[0]["pred_labels"][idx]
                class_name = self.label_mapping.get(class_id, "unknown")
                marker.text = class_name
                marker_array.markers.append(marker)

                detection = Detection3D()
                detection.header = pointcloud_msg.header
                detection.bbox.center.position.x = float(box[0])
                detection.bbox.center.position.y = float(box[1])
                detection.bbox.center.position.z = float(box[2])
                detection.bbox.size.x = float(box[3])
                detection.bbox.size.y = float(box[4])
                detection.bbox.size.z = float(box[5])
                detected_object = ObjectHypothesisWithPose()
                detected_object.hypothesis.class_id = str(pred_dicts[0]["pred_labels"][idx])
                detected_object.hypothesis.score = float(pred_dicts[0]["pred_scores"][idx])
                detection.results.append(detected_object)
                detections.detections.append(detection)
        if self.publish_detection:
            self.viz_publisher.publish(marker_array)
        self.detections_publisher.publish(detections)

    def parse_config(self):
        parser = argparse.ArgumentParser(description="arg parser")
        parser.add_argument(
            "--cfg_file",
            type=str,
            default=self.model_config_path,
            help="specify the config for demo",
        )
        parser.add_argument(
            "--ckpt", type=str, default=self.model_path, help="specify the pretrained model"
        )

        args, _ = parser.parse_known_args()
        cfg_from_yaml_file(args.cfg_file, cfg)
        return args, cfg


class LidarDatalodaer(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, logger=None, ext=".bin"):
        super().__init__(dataset_cfg=dataset_cfg, class_names=class_names, training=training, logger=logger)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
