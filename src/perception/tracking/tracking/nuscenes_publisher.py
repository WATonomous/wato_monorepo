import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D
from geometry_msgs.msg import PoseWithCovariance
from tracking_msgs.msg import TrackedObstacleList
from std_msgs.msg import Header

from nuscenes.nuscenes import NuScenes
from nuscenes.utils.geometry_utils import transform_matrix
from pyquaternion import Quaternion

from nusc_viz_3d import NuscViz

import numpy as np
import time
import os
import json

class NuScenesPublisher(Node):
    def __init__(self):
        super().__init__('nuscenes_publisher')
        self.publisher = self.create_publisher(Detection3DArray, 'detections', 10)

        self.tracked_obstacles_sub = self.create_subscription(
            TrackedObstacleList,  # Message type of the tracked obstacles
            "tracked_obstacles",  # Topic name
            self.tracked_obstacles_callback,
            10  # Queue size
        )

        # Load nuScenes
        self.nusc = NuScenes(version='v1.0-mini', dataroot=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_tracking", "nuscenes"), verbose=True)
        self.viz = NuscViz(self.nusc)
        # Get the first scene
        self.scene_index = 0
        self.scene = self.nusc.scene[self.scene_index]
        self.scene_name = self.scene['name']
        self.scene_token = self.scene['token']
        self.sample_token = self.scene['first_sample_token']
        self.samples = []
        self.timer_period = 0.5  # seconds

        self.timer = self.create_timer(self.timer_period, self.publish_next_sample)

        self.results = {"results": {}, "meta": {"use_camera": True,
                                                "use_lidar": True,
                                                "use_radar": False,}}

        self.name_convert = {
            'human.pedestrian.adult': 'pedestrian',
            'human.pedestrian.child': 'pedestrian',
            'human.pedestrian.construction_worker': 'pedestrian',
            'human.pedestrian.personal_mobility': 'pedestrian',
            'human.pedestrian.police_officer': 'pedestrian',
            'human.pedestrian.stroller': 'pedestrian',
            'human.pedestrian.wheelchair': 'pedestrian',
            'vehicle.car': 'car',
            'vehicle.bus.bendy': 'bus',
            'vehicle.bus.rigid': 'bus',
            'vehicle.truck': 'truck',
            'vehicle.trailer': 'trailer',
            'vehicle.construction': 'truck',
            'vehicle.motorcycle': 'motorcycle',
            'vehicle.bicycle': 'bicycle',
            'movable_object.barrier': 'bus',
            'movable_object.trafficcone': 'bicycle',
            'movable_object.pushable_pullable': 'trailer',
            'movable_object.debris': 'pedestrian',
            'static_object.bicycle_rack': 'bicycle'
        }

        self.latest_gts = []
        self.latest_trks = []

        self.acc_error = 0
        self.num_data_pts = 0

        self.sent = 0
        self.received = 0

    def init_scene(self):
        self.scene = self.nusc.scene[self.scene_index]
        self.scene_token = self.scene['token']
        self.sample_token = self.scene['first_sample_token']

    def publish_next_sample(self):
        self.latest_gts = []

        if self.sample_token == "":
            # if self.scene_index < len(self.nusc.scene) - 1:
            #     self.scene_index += 1
            #     self.init_scene()
            #     print(f"Scene {self.scene_index + 1} loaded")
            #     return
            
            if self.received < self.sent:
                return
                
            self.get_logger().info("No more samples.")
            print((self.acc_error/self.num_data_pts)**0.5)
            self.viz.save_frames_to_video()

            # res_dest = os.path.join(os.path.dirname(os.path.abspath(__file__)), "track_results")
            # self.get_logger().info(f"Results saved to {res_dest}")
            # res_file = f"results{len(os.listdir(res_dest))}.json"
            # with open(os.path.join(res_dest, res_file), 'w') as rj:
            #     json.dump(self.results, rj)

            self.destroy_timer(self.timer)
            return

        sample = self.nusc.get('sample', self.sample_token)
        self.samples.append(sample)
        msg = Detection3DArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        for ann_token in sample['anns']:
            ann = self.nusc.get('sample_annotation', ann_token)
            self.latest_gts.append([float(p) for p in ann['translation'][:3]])

            det = Detection3D()
            det.header = msg.header

            # Label and confidence
            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = ann['category_name']
            hypo.hypothesis.score = 1.0  # nuScenes doesn't have scores
            hypo.pose.pose.position.x = float(ann['translation'][0])
            hypo.pose.pose.position.y = float(ann['translation'][1])
            hypo.pose.pose.position.z = float(ann['translation'][2])
            q = Quaternion(ann['rotation'])
            hypo.pose.pose.orientation.x = q.x
            hypo.pose.pose.orientation.y = q.y
            hypo.pose.pose.orientation.z = q.z
            hypo.pose.pose.orientation.w = q.w

            det.results.append(hypo)

            # Bounding box
            bbox = BoundingBox3D()
            bbox.center.position = hypo.pose.pose.position
            bbox.center.orientation = hypo.pose.pose.orientation
            bbox.size.x = float(ann['size'][0])
            bbox.size.y = float(ann['size'][1])
            bbox.size.z = float(ann['size'][2])

            det.bbox = bbox
            msg.detections.append(det)

        self.sent += 1
        self.sample_token = sample['next']

        self.get_logger().info(f"Published sample {self.sample_token} from scene {self.scene_index + 1}")
        self.publisher.publish(msg)
        

    def tracked_obstacles_callback(self, msg: TrackedObstacleList):
        # print(self.acc_error)
        # viz
        self.received = msg.tracked_obstacles[0].obstacle.object_id
        print(f"Sent: {self.sent}; Received: {self.received}")

        if self.received - 1 >= 0:
            self.viz.process_sample(sample=self.samples[self.received-1], tracks=msg, frame_index=self.received-1, scene_name=self.scene_name)
        
        self.results["results"][self.scene_token] = []
        for trk_ob in msg.tracked_obstacles:
            ob = trk_ob.obstacle
            self.latest_trks.append([ob.pose.pose.position.x, ob.pose.pose.position.y, ob.pose.pose.position.z])
            if ob.label == "filler":
                self.latest_trks[-1] = None
                continue
            try:
                trk_name = self.name_convert[ob.label]
            except KeyError:
                with open(os.path.join(os.path.abspath(os.path.dirname(__file__)), "keys.txt"), 'a') as f:
                    f.write(ob.label + "\n")
                trk_name = "bicycle"

            self.results["results"][self.scene_token].append({"sample_token": self.sample_token,
                                                              "translation": [ob.pose.pose.position.x,
                                                                              ob.pose.pose.position.y,
                                                                              ob.pose.pose.position.z],
                                                              "size": [ob.width_along_x_axis,
                                                                       ob.depth_along_z_axis,
                                                                       ob.height_along_y_axis],
                                                              "rotation": [ob.pose.pose.orientation.w,
                                                                           ob.pose.pose.orientation.x,
                                                                           ob.pose.pose.orientation.y,
                                                                           ob.pose.pose.orientation.z],
                                                              "velocity": [ob.twist.twist.linear.x,
                                                                           ob.twist.twist.linear.z],
                                                              "tracking_id": ob.object_id,
                                                              "tracking_name": self.name_convert[ob.label],
                                                              "tracking_score": ob.confidence})
            
        for i in range(len(self.latest_trks)):
            if i < len(self.latest_gts) and self.latest_trks[i] != None:
                for j in range(3):
                    self.acc_error += (self.latest_gts[i][j] - self.latest_trks[i][j])**2
                self.num_data_pts += 1


def main(args=None):
    rclpy.init(args=args)
    node = NuScenesPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()