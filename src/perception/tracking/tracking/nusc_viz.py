import numpy as np
import matplotlib.pyplot as plt
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud, Box
from nuscenes.utils.geometry_utils import view_points, transform_matrix
from pyquaternion import Quaternion
import cv2
import os


SELECTED_BOX_NAMES = [
    #'vehicle.motorcycle',
    #'movable_object.barrier',
    'movable_object.pushable_pullable',
    #'human.pedestrian.adult',
    #'movable_object.trafficcone',
    #'vehicle.bicycle',
    'vehicle.truck',
    'vehicle.car',
    #'vehicle.bus.rigid',
    #'vehicle.construction',
]

COLORS = {
    'RED' : (255, 0, 0),
    'GREEN' : (0, 255, 0),
    'BLUE' : (0, 0, 255),
    'YELLOW' : (255, 255, 0),
    'LIGHT_GRAY' : (127, 127, 127),
    'WHITE' : (255, 255, 255),
}

BBOX_COLORS = {
    #'vehicle.motorcycle' : ,
    'movable_object.barrier' : 'LIGHT_GRAY',
    'movable_object.pushable_pullable' : 'LIGHT_GRAY',
    'human.pedestrian.adult' : 'RED',
    'movable_object.trafficcone' : 'LIGHT_GRAY',
    #'vehicle.bicycle',
    'vehicle.truck' : 'GREEN',
    'vehicle.car' : 'BLUE',
    #'vehicle.bus.rigid',
    'vehicle.construction' : 'YELLOW',
}


class NuscViz:
    def __init__(self, nu=None, cam_name="CAM_FRONT", w=1600, h=900, fps=2, bev_sz=800, bev_rg=40, vfmt="mp4", codec="mp4v"):
        # Video settings
        self.camera_name = cam_name
        self.video_width = w
        self.video_height = h
        self.bev_img_size = bev_sz
        self.bev_range = bev_rg # meters in each direction

        fourcc = cv2.VideoWriter_fourcc(*codec)
        self.video_name = f"video_{self.camera_name}.{vfmt}"
        self.video_writer = cv2.VideoWriter(self.video_name, fourcc, fps, (self.video_width, self.video_height))

        # Nusc stuff
        if nu is None:
            self.nusc = NuScenes(version='v1.0-mini', dataroot=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_tracking", "nuscenes"), verbose=True)
        else:
            self.nusc = nu
        
        
    def get_cv2_color(self, category_name, returnBGR = True):
        # color = (255, 255, 255)
        # if 'pedestrian' in category_name:
        #     color = (255, 0, 0) # RED
        # elif 'truck' in category_name:
        #     color = (0, 255, 0) # GREEN
        # elif 'car' in category_name:
        #     color = (0, 0, 255) # BLUE
        # elif 'construction' in category_name:
        #     color = (255, 255, 0) # YELLOW
        # elif 'movable_object' in category_name:
        #     color = (127, 127, 127) # LIGHT GREY
        # else:
        #     color = (255, 255, 255) # WHITE
        if category_name[0] == '_':
            color = COLORS['RED']
        else:
            try:
                color = COLORS[BBOX_COLORS[category_name]]
            except KeyError:
                color = COLORS['WHITE']

        if returnBGR:
            color = (color[2], color[1], color[0])
        return color


    # Helper function to apply 4x4 transform matrix to a Box
    def box_transform(self, box, transform):
        """
        Apply a 4x4 transform matrix to a Box.
        Returns a new transformed Box.
        """
        translation = transform[:3, 3]
        rotation = Quaternion(matrix=transform[:3, :3])
        
        new_box = box.copy()

        new_box.rotate(rotation)
        new_box.translate(translation)
        return new_box
    
    def get_transforms(self, cam_data, lidar_data):
        # Extract data
        cam_cs = self.nusc.get('calibrated_sensor', cam_data['calibrated_sensor_token'])
        lidar_cs = self.nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
        
        cam_pose = self.nusc.get('ego_pose', cam_data['ego_pose_token'])
        lidar_pose = self.nusc.get('ego_pose', lidar_data['ego_pose_token'])

        # Camera transform matrices
        cam_global_to_ego = transform_matrix(cam_pose['translation'],
                                             Quaternion(cam_pose['rotation']),
                                             inverse=True)
        ego_to_cam = transform_matrix(cam_cs['translation'],
                                      Quaternion(cam_cs['rotation']),
                                      inverse=True)
        global_to_cam = ego_to_cam @ cam_global_to_ego

        # Lidar transform matrices
        lidar_global_to_ego = transform_matrix(lidar_pose['translation'],
                                               Quaternion(lidar_pose['rotation']),
                                               inverse=True)
        ego_to_lidar = transform_matrix(lidar_cs['translation'],
                                        Quaternion(lidar_cs['rotation']),
                                        inverse=True)
        global_to_lidar = ego_to_lidar @ lidar_global_to_ego

        lidar_to_cam = ego_to_cam @ np.linalg.inv(ego_to_lidar)

        return global_to_cam, global_to_lidar, lidar_to_cam


    
    def process_scene(self, scene_index):
        # Get first sample
        scene = self.nusc.scene[scene_index]
        scene_name = scene['name']
        relative_index = 0
        sample_token = scene['first_sample_token']
        sample = self.nusc.get('sample', sample_token)

        # Process samples in scene
        while sample_token != "":
            sample = self.nusc.get('sample', sample_token)
            self.process_sample(sample=sample, frame_index=relative_index, scene_name=scene_name)
            sample_token = sample['next']
            relative_index += 1

        return


    def process_sample(self, sample=None, tracks=None, frame_index=-1, sample_token="", scene_name="NA", show_pc_on_cam=False):
        if sample is None:
            if frame_index >= 0:
                sample = self.nusc.sample[frame_index]
            else:
                sample = self.nusc.get('sample', sample_token)

        # Get ground truth bboxes (global frame)
        gt_boxes = []
        for ann_token in sample['anns']:
            gt_boxes.append(self.nusc.get_box(ann_token))

        tracked_boxes = []
        if tracks is not None:
            for tr in tracks.tracked_obstacles:
                ob = tr.obstacle
                ps = ob.pose.pose.position
                ot = ob.pose.pose.orientation
                b = Box(
                    np.array([ps.x, ps.y, ps.z]),
                    np.array([ob.width_along_x_axis, ob.height_along_y_axis, ob.depth_along_z_axis]),
                    Quaternion(ot.w, ot.x, ot.y, ot.z),
                    name=f"_{ob.label}"
                )
                tracked_boxes.append(b)

        # --- Get camera and lidar info ---
        cam_token = sample['data'][self.camera_name]
        lidar_token = sample['data']['LIDAR_TOP']
        
        cam_data = self.nusc.get('sample_data', cam_token)
        lidar_data = self.nusc.get('sample_data', lidar_token)

        cam_cs = self.nusc.get('calibrated_sensor', cam_data['calibrated_sensor_token'])
        camera_intrinsic = np.array(cam_cs['camera_intrinsic'])

        global_to_cam, global_to_lidar, lidar_to_cam = self.get_transforms(cam_data, lidar_data)

        # --- Get underlying lidar image ---
        # Load point cloud
        pc = LidarPointCloud.from_file(self.nusc.get_sample_data_path(lidar_token))

        lidar_points = pc.points[:2, :]
        # Scale and shift lidar_points to image coordinates
        bev = np.zeros((self.bev_img_size, self.bev_img_size, 3), dtype=np.uint8)
        scale = self.bev_img_size / (2 * self.bev_range)
        x_img = np.int32(np.clip((lidar_points[0] + self.bev_range) * scale, 0, self.bev_img_size - 1))
        y_img = np.int32(np.clip((lidar_points[1] + self.bev_range) * scale, 0, self.bev_img_size - 1))
        for x, y in zip(x_img, y_img):
            cv2.circle(bev, (x, self.bev_img_size - y), 1, (128, 128, 128), -1)
            
        # --- Get underlying camera image ---
        # Load camera image
        image_path = self.nusc.get_sample_data_path(cam_token)
        image = cv2.imread(image_path)
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w, _ = image.shape

        if show_pc_on_cam:
            # Transform pc frame from lidar to camera
            pc.transform(lidar_to_cam)

            # Project to 2D
            cam_points = pc.points[:3, :]  # x, y, z in camera frame
            points_2d = view_points(cam_points, camera_intrinsic, normalize=True)

            # Filter out points not in frame
            mask = (cam_points[2, :] > 0) & \
                (points_2d[0, :] >= 0) & (points_2d[0, :] < w) & \
                (points_2d[1, :] >= 0) & (points_2d[1, :] < h)

            valid_points = points_2d[:, mask]

            # Draw points on image
            for i in range(valid_points.shape[1]):
                x, y = int(valid_points[0, i]), int(valid_points[1, i])
                cv2.circle(image, (x, y), 2, (0, 255, 0), -1)

        # --- Draw bboxes on lidar image ---
        for boxes in [gt_boxes, tracked_boxes]:
            for box in boxes:
                # Transform box to lidar frame
                lidar_box = self.box_transform(box, global_to_lidar)

                # Get bottom corners of box to represent bev
                corners = lidar_box.bottom_corners()[:2, :]  # shape (2,4)

                # Convert to pixel coordinates
                x_pix = np.int32(np.clip((corners[0] + self.bev_range) * scale, 0, self.bev_img_size - 1))
                y_pix = np.int32(np.clip((corners[1] + self.bev_range) * scale, 0, self.bev_img_size - 1))
                y_pix = self.bev_img_size - y_pix  # flip y for image display

                # Draw bounding box edges
                color = self.get_cv2_color(box.name)
                for i in range(4):
                    pt1 = (x_pix[i], y_pix[i])
                    pt2 = (x_pix[(i + 1) % 4], y_pix[(i + 1) % 4])
                    cv2.line(bev, pt1, pt2, color, 2)  # red lines

                # Draw front direction line
                center = np.mean(np.column_stack((x_pix, y_pix)), axis=0).astype(int)
                front_center = np.mean(np.column_stack((x_pix[0:2], y_pix[0:2])), axis=0).astype(int)
                cv2.line(bev, tuple(center), tuple(front_center), (255, 255, 0), 2)

        # --- Draw bboxes on camera image ---
        top_boxes = []
        for boxes in [gt_boxes, tracked_boxes]:
            box_entries = [] # List of (box, projected_corners_2d, area)
            for box in boxes:
                # Transform box to camera frame
                cam_box = self.box_transform(box, global_to_cam)
                
                # Ignore boxes behind camera
                if cam_box.center[2] <= 0:
                    continue

                # Project 3D to 2D
                corners_3d = cam_box.corners()
                corners_2d = view_points(corners_3d, camera_intrinsic, normalize=True)

                # Correct x and y flipping when z negative
                for pp in range(8):
                    if corners_3d[2, pp] < 0:
                        corners_2d[0, pp] = w - corners_2d[0, pp]
                        corners_2d[1, pp] = h - corners_2d[1, pp]

                # Filter out objects completely out of frame
                if not ((corners_2d[0] >= 0) & (corners_2d[0] < w) & 
                        (corners_2d[1] >= 0) & (corners_2d[1] < h)).any():
                    continue

                # Compute area in pixels
                min_x, max_x = corners_2d[0].min(), corners_2d[0].max()
                min_y, max_y = corners_2d[1].min(), corners_2d[1].max()
                area = (max_x - min_x) * (max_y - min_y)

                box_entries.append((box, corners_2d, area))

            # Get bboxes with largest area
            top_boxes.append(sorted(box_entries, key=lambda x: x[2], reverse=True)[:20])

        # Vertices of each edge
        edge_idx = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7)
        ]

        for top in top_boxes:
            for box, corners_2d, area in top:
                color = self.get_cv2_color(box.name)
                if box.name in SELECTED_BOX_NAMES or box.name[1:] in SELECTED_BOX_NAMES:
                    # Draw label
                    cx = int(corners_2d[0].mean())
                    cy = int(corners_2d[1].mean())
                    cv2.putText(image, box.name.split('.')[-1], (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    # Draw edges
                    for i, j in edge_idx:
                        pt1 = (int(corners_2d[0, i]), int(corners_2d[1, i]))
                        pt2 = (int(corners_2d[0, j]), int(corners_2d[1, j]))
                        cv2.line(image, pt1, pt2, color=color, thickness=2)

        # Combine horizontally
        if bev.shape[0] != h:
            bev = cv2.resize(bev, (h, h))

        cv2.putText(image, f"CAMERA VIEW: {self.camera_name} + PC + GT_BOX", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(bev, f"BEV: LIDAR_TOP + GT_BOX", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        combined = np.hstack((image, bev))
        cv2.imwrite(os.path.join(os.path.dirname(os.path.abspath(__file__)), "viz_frames", f"scene_{scene_name}_frame_{frame_index}_{self.camera_name}.png"), combined)
        frame = cv2.resize(combined, (self.video_width, self.video_height))
        # video_writer.write(frame)
        print(f"Processed frame {frame_index} of scene {scene_name} for {self.camera_name}.")
        self.video_writer.write(frame)
        # --- Show the image ---
        #plt.figure(figsize=(16, 9))
        #plt.imshow(image)
        #plt.title("LiDAR points projected to camera image")
        #plt.axis("off")
        #plt.savefig(f'frame_{frame_index}_{CAMERA_NAME}.png')
        return frame

    def save_frames_to_video(self):
        self.video_writer.release()
        print(f"Video saved as {self.video_name}")
