import numpy as np
import matplotlib.pyplot as plt
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud
from nuscenes.utils.geometry_utils import view_points, transform_matrix
from pyquaternion import Quaternion
import cv2
import os

FRAME_INDEXES = range(0, 50)
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

VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_FPS = 1600, 900, 2  #


# Helper to move box to camera frame
def to_camera_frame(box):
    # global → vehicle
    box.translate(-t_v)
    box.rotate(Qv2g.inverse)
    # vehicle → camera
    box.translate(-t_c)
    box.rotate(Qc2v)
    return box

def get_cv2_color(category_name, returnBGR = True):
    color = (255, 255, 255)
    if 'pedestrian' in category_name:
        color = (255, 0, 0)  # RED
    elif 'truck' in category_name:
        color = (0, 255, 0) # GREEN
    elif 'car' in category_name:
        color = (0, 0, 255) # BLUE
    elif 'contruction' in category_name:
        color = (255, 255, 0)  # YELLOW
    elif 'movable_object' in category_name:
        color = (127, 127, 127)  # LIGHT GREY
    else:
        color = (255, 255, 255)  # WHITE

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

# Sample token
for frame_index in FRAME_INDEXES:
    sample = nusc.sample[frame_index]

    # Get camera and lidar data tokens
    cam_token = sample['data'][CAMERA_NAME]
    lidar_token = sample['data']['LIDAR_TOP']

    cam_data = nusc.get('sample_data', cam_token)
    lidar_data = nusc.get('sample_data', lidar_token)

    # Load image
    image_path = nusc.get_sample_data_path(cam_token)
    image = cv2.imread(image_path)
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    h, w, _ = image.shape

    # Load point cloud
    pc = LidarPointCloud.from_file(nusc.get_sample_data_path(lidar_token))

    # --- Step 1: Transform LiDAR to ego vehicle frame ---
    cs_lidar = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
    pc.rotate(Quaternion(cs_lidar['rotation']).rotation_matrix)
    pc.translate(np.array(cs_lidar['translation']))

    # --- Step 2: Transform ego vehicle to camera frame ---
    cs_cam = nusc.get('calibrated_sensor', cam_data['calibrated_sensor_token'])
    pc.translate(-np.array(cs_cam['translation']))
    pc.rotate(Quaternion(cs_cam['rotation']).rotation_matrix.T)

    # --- Step 3: Project to image plane ---
    points_cam = pc.points[:3, :]  # x, y, z in camera frame
    camera_intrinsic = np.array(cs_cam['camera_intrinsic'])

    # Project
    points_2d = view_points(points_cam, camera_intrinsic, normalize=True)

    # --- Step 4: Filter points in front of camera and inside image ---
    mask = (points_cam[2, :] > 0) & \
        (points_2d[0, :] >= 0) & (points_2d[0, :] < w) & \
        (points_2d[1, :] >= 0) & (points_2d[1, :] < h)

    # Get valid 2D points
    valid_points = points_2d[:, mask]

    # --- Step 5: Draw on image ---
    for i in range(valid_points.shape[1]):
        x, y = int(valid_points[0, i]), int(valid_points[1, i])
        cv2.circle(image, (x, y), 2, (0, 255, 0), -1)

    lidar_points = LidarPointCloud.from_file(nusc.get_sample_data_path(lidar_token)).points[:2, :]
    bev_img_size = 800  # 800x800 pixels
    bev_range = 40  # meters in each direction
    # Scale and shift lidar_points to image coordinates
    bev = np.zeros((bev_img_size, bev_img_size, 3), dtype=np.uint8)
    scale = bev_img_size / (2 * bev_range)
    x_img = np.int32(np.clip((lidar_points[0] + bev_range) * scale, 0, bev_img_size - 1))
    y_img = np.int32(np.clip((lidar_points[1] + bev_range) * scale, 0, bev_img_size - 1))
    for x, y in zip(x_img, y_img):
        cv2.circle(bev, (x, bev_img_size - y), 1, (128, 128, 128), -1)  
    
    # Get transforms
    cs_record = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
    pose_record = nusc.get('ego_pose', lidar_data['ego_pose_token'])

    # Convert quaternion format [w, x, y, z] to [x, y, z, w]
    pose_rot = pose_record['rotation']
    pose_q = Quaternion(pose_rot)
    cs_rot = cs_record['rotation']
    cs_q = Quaternion(cs_rot)

    # Get transform matrices
    global_to_ego = transform_matrix(pose_record['translation'], pose_q, inverse=True)
    ego_to_sensor = transform_matrix(cs_record['translation'], cs_q, inverse=True)
    global_to_sensor = ego_to_sensor @ global_to_ego

    # Get bounding boxes in global coordinates
    boxes = nusc.get_boxes(lidar_data['token'])

    for box in boxes:
        # Transform box to sensor (LiDAR) frame
        box_sensor = box_transform(box, global_to_sensor)

        # Get bottom corners of box (BEV)
        corners = box_sensor.bottom_corners()[:2, :]  # shape (2,4)

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

    # Precompute transforms
    cs = nusc.get('calibrated_sensor', cam_data['calibrated_sensor_token'])
    pose = nusc.get('ego_pose', cam_data['ego_pose_token'])
    Qc2v = Quaternion(cs['rotation']).inverse   # camera frame ← sensor frame
    Qv2g = Quaternion(pose['rotation'])        # global frame ← vehicle frame
    t_c = np.array(cs['translation'])
    t_v = np.array(pose['translation'])

    boxes = nusc.get_boxes(cam_data['token'])
    # List of (box, projected_corners_2d, area)
    box_entries = []
    for box in boxes:
        # ignore boxes behind camera
        if box.center[2] <= 0:
            continue

        # transform & project
        box_cam = to_camera_frame(box.copy())        # work on a copy
        corners_3d = box_cam.corners()               # 3×8
        corners_2d = view_points(corners_3d, camera_intrinsic, normalize=True)

        # filter out-of-image entirely
        if not ((corners_2d[0] >= 0) & (corners_2d[0] < w) & 
                (corners_2d[1] >= 0) & (corners_2d[1] < h)).any():
            continue

        # compute AABB in pixels
        min_x, max_x = corners_2d[0].min(), corners_2d[0].max()
        min_y, max_y = corners_2d[1].min(), corners_2d[1].max()
        area = (max_x - min_x) * (max_y - min_y)

        box_entries.append((box, corners_2d, area))

    # pick top 10 by area
    top10 = sorted(box_entries, key=lambda x: x[2], reverse=True)[:100]

    # draw them
    out = image.copy()

    edge_idx = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]

    for box, corners_2d, area in top10:
        # choose a color (e.g., green)
        color = get_cv2_color(box.name)
        if box.name in SELECTED_BOX_NAMES:
            # draw label at box center
            cx = int(corners_2d[0].mean())
            cy = int(corners_2d[1].mean())
            cv2.putText(out, box.name.split('.')[-1], (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        else:
            #print(f'====Ignore {box.name}')
            continue

        # draw edges
        if box.name == "vehicle.truck":
            print(corners_3d)
        for i, j in edge_idx:
            pt1 = (int(corners_2d[0, i]), int(corners_2d[1, i]))
            pt2 = (int(corners_2d[0, j]), int(corners_2d[1, j]))
            cv2.line(out, pt1, pt2, color=color, thickness=2)

    # Combine horizontally
    if bev.shape[0] != out.shape[0]:
        bev = cv2.resize(bev, (out.shape[0], out.shape[0]))

    cv2.putText(out, f"CAMERA VIEW: {CAMERA_NAME} + PC + GT_BOX", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(bev, f"BEV: LIDAR_TOP + GT_BOX", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    
    combined = np.hstack((out, bev))
    cv2.imwrite(os.path.join(os.path.dirname(os.path.abspath(__file__)), "viz_frames", f"frame_{frame_index}_{CAMERA_NAME}.png"), combined)
    frame = cv2.resize(combined, (VIDEO_WIDTH, VIDEO_HEIGHT))
    video_writer.write(frame)
    print(f"Processed frame {frame_index} for {CAMERA_NAME}.")
    # --- Show the image ---
    #plt.figure(figsize=(16, 9))
    #plt.imshow(out)
    #plt.title("LiDAR points projected to camera image")
    #plt.axis("off")
    #plt.savefig(f'frame_{frame_index}_{CAMERA_NAME}.png')

video_writer.release()
print(f"Video saved video_{CAMERA_NAME}.mp4")
