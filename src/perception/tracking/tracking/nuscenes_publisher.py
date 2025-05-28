import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D
from geometry_msgs.msg import PoseWithCovariance
from tracking_msgs.msg import TrackedObstacleList
from std_msgs.msg import Header

from nuscenes.nuscenes import NuScenes
from nuscenes.utils.geometry_utils import view_points, transform_matrix
from pyquaternion import Quaternion

from nusc_viz_3d import NuscViz, box_transform, get_sensor_info
from cv_bridge import CvBridge

import numpy as np
import time
import os
import json
from random import gauss, uniform

class NuScenesPublisher(Node):
    def __init__(self):
        super().__init__('nuscenes_publisher')

        self.declare_parameter('det_topic', "detections")
        self.declare_parameter('img_topic', '/annotated_image')
        self.declare_parameter('track_topic', "tracked_obstacles")
        
        self.det_topic = self.get_parameter('det_topic').value
        self.img_topic = self.get_parameter('img_topic').value
        self.track_topic = self.get_parameter('track_topic').value

        self.bridge = CvBridge()
        self.nusc_pub = self.create_publisher(Detection3DArray, self.det_topic, 10)
        self.img_pub = self.create_publisher(Image, self.img_topic, 10)
        self.tracked_obstacles_sub = self.create_subscription(
            TrackedObstacleList,
            self.track_topic,
            self.tracked_obstacles_callback,
            10
        )

        data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_tracking", "nuscenes")
        with open(os.path.join(data_path, "dataset_name.txt"), 'r') as f:
            default_set = f.readline().rstrip('\n')
        
        self.declare_parameter('dataset_name', default_set)
        self.dataset_name = self.get_parameter('dataset_name').value

        # Load nuScenes
        self.nusc = NuScenes(version=self.dataset_name, dataroot=os.path.join(data_path, f"{self.dataset_name}_data"), verbose=True)
        self.viz = NuscViz(self.nusc)
        # Get the first scene
        self.scene_index = 0
        self.scene = self.nusc.scene[self.scene_index]
        self.scene_name = self.scene['name']
        self.scene_token = self.scene['token']
        self.sample_token = self.scene['first_sample_token']
        self.samples = []

        self.declare_parameter('timer_period', 1) # seconds
        self.timer_period = self.get_parameter('timer_period').value

        self.timer = self.create_timer(self.timer_period, self.publish_next_sample)

        self.results = {"results": {}, "meta": {"use_camera": True,
                                                "use_lidar": True,
                                                "use_radar": False,}}

        # surjective
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

        self.declare_parameter('pub_noise', False)
        self.declare_parameter('simulate_occlusion', True)
        self.declare_parameter('occ_threshold', 0.97)
        
        self.pub_noise = self.get_parameter('pub_noise').value
        self.simulate_occlusion = self.get_parameter('simulate_occlusion').value
        self.occ_threshold = self.get_parameter('occ_threshold').value

    def init_scene(self):
        self.scene = self.nusc.scene[self.scene_index]
        self.scene_token = self.scene['token']
        self.sample_token = self.scene['first_sample_token']

    def publish_next_sample(self):
        self.latest_gts = []

        if self.sample_token == "":
            # todo: uncomment if iterating through all scenes instead of just one
            # if self.scene_index < len(self.nusc.scene) - 1:
            #     self.scene_index += 1
            #     self.init_scene()
            #     print(f"Scene {self.scene_index + 1} loaded")
            #     return
            
            if self.received < self.sent:
                return
                
            self.get_logger().info("No more samples.")
            # print((self.acc_error/self.num_data_pts)**0.5)
            self.viz.save_frames_to_video()

            # todo: uncomment when get actual test data
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

        cam_info = get_sensor_info(self.nusc, sample, 'CAM_FRONT')
        boxes = [[box, None] for box in self.nusc.get_boxes(cam_info['data']['token'])]
        ego = cam_info['ego']
        if self.simulate_occlusion:
            # front = {}
            using = []

            # Sort by absolute z diff from sensor
            
            cs = cam_info['cs']
            global_to_ego = transform_matrix(ego['translation'],
                                               Quaternion(ego['rotation']),
                                               inverse=True)
            ego_to_cam = transform_matrix(cs['translation'],
                                      Quaternion(cs['rotation']),
                                      inverse=True)
            global_to_cam = ego_to_cam @ global_to_ego

            for b in boxes:
                b[1] = box_transform(b[0], global_to_cam)
            boxes = sorted(boxes, key=lambda x: abs(x[1].center[2]))

        # todo: make publisher not publish ground truth (get real data)
        # newb = []
        for box_tuple in boxes:
            box = box_tuple[0]
            box_rel = box_tuple[1]

            if self.simulate_occlusion:
                cnrs_3d = box_rel.corners()
                cnrs = view_points(cnrs_3d, cam_info['intrinsic'], normalize=True)
                x1 = min(cnrs[0])
                x2 = max(cnrs[0])
                y1 = min(cnrs[1])
                y2 = max(cnrs[1])
                
                if box_rel.center[2] <= 0:
                    continue

                occed = False
                for u in using:
                    x3 = u[0]
                    x4 = u[1]
                    y3 = u[2]
                    y4 = u[3]
                    overlap = max(0, min(x2, x4) - max(x1, x3)) * max(0, min(y2, y4) - max(y1, y3))
                    area = (x2 - x1) * (y2 - y1)

                    percent_occ = overlap/area
                    if percent_occ > self.occ_threshold:
                        occed = True
                        break
                
                if occed:
                    continue
                elif min(cnrs_3d[2]) > 0:
                    using.append([x1, x2, y1, y2, box.name])


            self.latest_gts.append([float(p) for p in box.center])

            if self.pub_noise:
                vol = box.wlh[0] * box.wlh[1] * box.wlh[2]
                miss_chance = (1 - min(0.995, vol**(0.1)*0.72))/2
                d_std = 0.2
                s_std = 0.02
            else:
                miss_chance = 0
                d_std = 0
                s_std = 0

            if uniform(0, 1) < miss_chance:
                # print(vol, miss_chance, "MISS")
                continue

            xd = gauss(0, d_std)
            yd = gauss(0, d_std)
            zd = gauss(0, d_std)
            xs = gauss(1, s_std)
            ys = gauss(1, s_std)
            zs = gauss(1, s_std)

            det = Detection3D()
            det.header = msg.header

            # Label and confidence
            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = box.name
            hypo.hypothesis.score = 1.0  # nuScenes doesn't have scores
            hypo.pose.pose.position.x = float(box.center[0]) + xd
            hypo.pose.pose.position.y = float(box.center[1]) + yd
            hypo.pose.pose.position.z = float(box.center[2]) + zd
            q = box.orientation
            hypo.pose.pose.orientation.x = q.x
            hypo.pose.pose.orientation.y = q.y
            hypo.pose.pose.orientation.z = q.z
            hypo.pose.pose.orientation.w = q.w

            det.results.append(hypo)

            # Bounding box
            bbox = BoundingBox3D()
            bbox.center.position = hypo.pose.pose.position
            bbox.center.orientation = hypo.pose.pose.orientation
            bbox.size.x = float(box.wlh[0]) * xs
            bbox.size.y = float(box.wlh[1]) * ys
            bbox.size.z = float(box.wlh[2]) * zs
            # bx = box.copy()
            # bx.center[0] = hypo.pose.pose.position.x
            # bx.center[1] = hypo.pose.pose.position.y
            # bx.center[2] = hypo.pose.pose.position.z
            # bx.wlh[0] = bbox.size.x
            # bx.wlh[1] = bbox.size.y
            # bx.wlh[2] = bbox.size.z
            # newb.append(bx)
            det.bbox = bbox
            msg.detections.append(det)
        # print(len(msg.detections))
        self.sent += 1
        self.sample_token = sample['next']

        # self.viz.process_sample(sample=sample, boxes=newb, frame_index=self.received-1, scene_name=self.scene_name)
        
        self.get_logger().info(f"Published sample {self.sample_token} from scene {self.scene_index + 1}")
        self.nusc_pub.publish(msg)
        

    def tracked_obstacles_callback(self, msg: TrackedObstacleList):
        # viz
        self.received = msg.tracked_obstacles[0].obstacle.object_id
        print(f"Sent: {self.sent}; Received: {self.received}")

        if self.received - 1 >= 0:
            # Append frame to video and also save as png
            cv_img = self.viz.process_sample(sample=self.samples[self.received-1], tracks=msg, frame_index=self.received-1, scene_name=self.scene_name)
            # Pub image for goxglove
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            self.get_logger().info("Published image")
            self.img_pub.publish(img_msg)
        
        # Update results dict
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
                                                              "tracking_name": trk_name,
                                                              "tracking_score": ob.confidence})
            
        # for i in range(len(self.latest_trks)):
        #     if i < len(self.latest_gts) and self.latest_trks[i] != None:
        #         for j in range(3):
        #             self.acc_error += (self.latest_gts[i][j] - self.latest_trks[i][j])**2
        #         self.num_data_pts += 1


def main(args=None):
    rclpy.init(args=args)
    node = NuScenesPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()