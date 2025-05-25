import numpy as np
import matplotlib.pyplot as plt
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud
from nuscenes.utils.geometry_utils import view_points, transform_matrix
from pyquaternion import Quaternion
import cv2
import os

FRAME_INDICES = range(0, 10)
CAMERA_NAME = 'CAM_FRONT'
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

VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_FPS = 1600, 900, 2


def get_cv2_color(category_name, returnBGR = True):
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

    try:
        color = COLORS[BBOX_COLORS[category_name]]
    except KeyError:
        color = COLORS['WHITE']

    if returnBGR:
        color = (color[2], color[1], color[0])
    return color


# Helper function to apply 4x4 transform matrix to a Box
def box_transform(box, transform):
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

# Load NuScenes
nusc = NuScenes(version='v1.0-mini', dataroot=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_tracking", "nuscenes"), verbose=True)


fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or 'XVID', 'avc1', etc.
video_writer = cv2.VideoWriter(f'video_{CAMERA_NAME}.mp4', fourcc, VIDEO_FPS, (VIDEO_WIDTH, VIDEO_HEIGHT))

def process_frame(frame_index, show_pc_on_cam=False):
    sample = nusc.sample[frame_index]

    # Get ground truth bboxes (global frame)
    boxes = []
    for ann_token in sample['anns']:
        boxes.append(nusc.get_box(ann_token))

    # --- Get camera and lidar info ---
    cam_token = sample['data'][CAMERA_NAME]
    lidar_token = sample['data']['LIDAR_TOP']

    cam_data = nusc.get('sample_data', cam_token)
    lidar_data = nusc.get('sample_data', lidar_token)

    cam_cs = nusc.get('calibrated_sensor', cam_data['calibrated_sensor_token'])
    lidar_cs = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])

    cam_pose = nusc.get('ego_pose', cam_data['ego_pose_token'])
    lidar_pose = nusc.get('ego_pose', lidar_data['ego_pose_token'])

    camera_intrinsic = np.array(cam_cs['camera_intrinsic'])

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

    
    # --- Get underlying lidar image ---
    # Lidar image config
    bev_img_size = 800  # 800x800 pixels
    bev_range = 40  # meters in each direction

    # Load point cloud
    pc = LidarPointCloud.from_file(nusc.get_sample_data_path(lidar_token))

    lidar_points = pc.points[:2, :]
    # Scale and shift lidar_points to image coordinates
    bev = np.zeros((bev_img_size, bev_img_size, 3), dtype=np.uint8)
    scale = bev_img_size / (2 * bev_range)
    x_img = np.int32(np.clip((lidar_points[0] + bev_range) * scale, 0, bev_img_size - 1))
    y_img = np.int32(np.clip((lidar_points[1] + bev_range) * scale, 0, bev_img_size - 1))
    for x, y in zip(x_img, y_img):
        cv2.circle(bev, (x, bev_img_size - y), 1, (128, 128, 128), -1)
        
    # --- Get underlying camera image ---
    # Load camera image
    image_path = nusc.get_sample_data_path(cam_token)
    image = cv2.imread(image_path)
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    h, w, _ = image.shape

    if show_pc_on_cam:
        # Transform pc frame from lidar to camera
        pc.transform(ego_to_cam @ np.linalg.inv(ego_to_lidar))

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
    for box in boxes:
        # Transform box to lidar frame
        lidar_box = box_transform(box, global_to_lidar)

        # Get bottom corners of box to represent bev
        corners = lidar_box.bottom_corners()[:2, :]  # shape (2,4)

        # Convert to pixel coordinates
        x_pix = np.int32(np.clip((corners[0] + bev_range) * scale, 0, bev_img_size - 1))
        y_pix = np.int32(np.clip((corners[1] + bev_range) * scale, 0, bev_img_size - 1))
        y_pix = bev_img_size - y_pix  # flip y for image display

        # Draw bounding box edges
        for i in range(4):
            pt1 = (x_pix[i], y_pix[i])
            pt2 = (x_pix[(i + 1) % 4], y_pix[(i + 1) % 4])
            cv2.line(bev, pt1, pt2, (0, 0, 255), 2)  # red lines

        # Draw front direction line
        center = np.mean(np.column_stack((x_pix, y_pix)), axis=0).astype(int)
        front_center = np.mean(np.column_stack((x_pix[0:2], y_pix[0:2])), axis=0).astype(int)
        cv2.line(bev, tuple(center), tuple(front_center), (255, 255, 0), 2)

    # --- Draw bboxes on camera image ---
    box_entries = [] # List of (box, projected_corners_2d, area)
    for box in boxes:
        # Transform box to camera frame
        cam_box = box_transform(box, global_to_cam)
        
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
    top = sorted(box_entries, key=lambda x: x[2], reverse=True)[:20]

    # Vertices of each edge
    edge_idx = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]

    for box, corners_2d, area in top:
        color = get_cv2_color(box.name)
        if box.name in SELECTED_BOX_NAMES:
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

    cv2.putText(image, f"CAMERA VIEW: {CAMERA_NAME} + PC + GT_BOX", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(bev, f"BEV: LIDAR_TOP + GT_BOX", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    
    combined = np.hstack((image, bev))
    cv2.imwrite(os.path.join(os.path.dirname(os.path.abspath(__file__)), "viz_frames", f"frame_{frame_index}_{CAMERA_NAME}.png"), combined)
    frame = cv2.resize(combined, (VIDEO_WIDTH, VIDEO_HEIGHT))
    # video_writer.write(frame)
    print(f"Processed frame {frame_index} for {CAMERA_NAME}.")
    return frame
    # --- Show the image ---
    #plt.figure(figsize=(16, 9))
    #plt.imshow(image)
    #plt.title("LiDAR points projected to camera image")
    #plt.axis("off")
    #plt.savefig(f'frame_{frame_index}_{CAMERA_NAME}.png')

for frame_index in FRAME_INDICES:
    video_writer.write(process_frame(frame_index))
video_writer.release()
print(f"Video saved video_{CAMERA_NAME}.mp4")
