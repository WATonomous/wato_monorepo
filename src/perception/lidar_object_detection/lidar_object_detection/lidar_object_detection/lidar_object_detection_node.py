import sys
import argparse
import rclpy
from rclpy.node import Node
import numpy as np
import torch
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import ObjectHypothesisWithPose, Detection3D, Detection3DArray
from sensor_msgs.msg import PointCloud2, PointField

# pylint: disable=wrong-import-position
sys.path.append("/home/bolty/OpenPCDet")
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils

class LidarObjectDetection(Node):
    def __init__(self):
        super().__init__('lidar_object_detection')
        self.declare_parameter("model_path", "/home/bolty/OpenPCDet/models/transfusion_trained_model.pth")
        self.declare_parameter("model_config_path", "/home/bolty/OpenPCDet/tools/cfgs/nuscenes_models/transfusion_lidar.yaml")
        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.model_path = self.get_parameter("model_path").value
        self.model_config_path = self.get_parameter("model_config_path").value
        self.lidar_data = self.get_parameter("lidar_topic").value

        self.viz_publisher = self.create_publisher(MarkerArray, "/lidar_detections_viz", 10)
        self.detections_publisher = self.create_publisher(Detection3DArray, "/lidar_detections", 10)

        self.subscription = self.create_subscription(
            PointCloud2, self.lidar_data, self.point_cloud_callback, 10
        )

        args, cfg = self.parse_config()
        self.logger = common_utils.create_logger()

        self.demo_dataset = LidarDataset(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
            logger=self.logger,
        )

        self.model = build_network(
            model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=self.demo_dataset
        )
        self.model.load_params_from_file(filename=args.ckpt, logger=self.logger, to_cpu=True)
        self.model.cuda()
        self.model.eval()

    def point_cloud_callback(self, msg):
        points = self.pointcloud2_to_xyz_array(msg)
        data_dict = {
            "points": points,
            "frame_id": msg.header.frame_id,
        }
        data_dict = self.demo_dataset.prepare_data(data_dict=data_dict)
        data_dict = self.demo_dataset.collate_batch([data_dict])
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
            if score > 0.7:
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

    def create_cloud_xyz32(self, header, points):
        """
        Create a sensor_msgs/PointCloud2 message from an array of points.

        :param header: std_msgs/Header, the header of the message.
        :param points: numpy.ndarray, an Nx3 array of xyz points.
        :return: sensor_msgs/PointCloud2, the constructed PointCloud2 message.
        """
        # Create fields for the PointCloud2 message
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        y_angle = np.deg2rad(90)
        z_angle = np.deg2rad(-90)
        rotation_matrix_y = np.array(
            [
                [np.cos(y_angle), 0, np.sin(y_angle)],
                [0, 1, 0],
                [-np.sin(y_angle), 0, np.cos(y_angle)],
            ]
        )
        rotation_matrix_z = np.array(
            [
                [np.cos(z_angle), -np.sin(z_angle), 0],
                [np.sin(z_angle), np.cos(z_angle), 0],
                [0, 0, 1],
            ]
        )

        points_np = points.cpu().numpy()
        points_np_y = np.dot(points_np, rotation_matrix_y)
        points_transformed = np.dot(points_np_y, rotation_matrix_z.T)

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = points_transformed.shape[0]
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = bool(np.isfinite(points_transformed).all())
        cloud.data = np.asarray(points_transformed, np.float32).tobytes()

        return cloud


class LidarDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, logger=None, ext=".bin"):
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, logger=logger
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
