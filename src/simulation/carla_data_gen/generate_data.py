import carla
import os
import queue
import numpy as np
import cv2
import time
import random
import yaml
import utils

image_queue = queue.Queue()

# Load sensor configuration from a YAML file
def load_sensor_config(config_path):
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

# Save image data to disk
def save_image(image, image_type, save_dir):
    print("Saving image")
    if image_type == "depth":
        image.save_to_disk(f'{save_dir}/%d_{image_type}.png' % image.frame)
    elif image_type == "semantic_segmentation":
        image.save_to_disk(f'{save_dir}/%d_{image_type}.png' % image.frame, carla.ColorConverter.CityScapesPalette)
    else:
        image.save_to_disk(f'{save_dir}/%d_{image_type}.png' % image.frame)
        image_queue.put(image)

# Process and save LiDAR data
def process_lidar_data(lidar_data, save_dir):
    # Get the LiDAR points. Each point is (x, y, z, intensity)
    points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    
    # Prepare to save the points in PCD format
    pcd_header = f'''# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
'''
    pcd_data = '\n'.join([' '.join(map(str, point)) for point in points])
    
    # Define file path
    file_path = f"{save_dir}/{lidar_data.frame}_lidar.pcd"

    # Open file and write headers and points
    with open(file_path, 'w') as pcd_file:
        pcd_file.write(pcd_header + pcd_data)

def spawn_npcs(world, blueprint_library, config, spawn_points):
    actor_list = []

    num_vehicles = config['npcs']['num_vehicles']
    num_walkers = config['npcs']['num_walkers']

    # Spawn vehicles
    for _ in range(num_vehicles):
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        vehicle_transform = random.choice(spawn_points)
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        actor_list.append(vehicle)
        spawn_points.remove(vehicle_transform)
    
    # Spawn walkers
    num_walkers_success = 0
    while num_walkers_success < num_walkers:
        walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
        walker_location = world.get_random_location_from_navigation()
        walker_transform = carla.Transform(walker_location)
        walker = world.try_spawn_actor(walker_bp, walker_transform)
        if walker is not None:
            actor_list.append(walker)
            num_walkers_success += 1
    return actor_list


# Spawn sensors based on YAML configuration
def spawn_sensors(world, blueprint_library, vehicle, config, save_dir):
    actor_list = []

    for cam_config in config['sensors']['camera']:
        cam_bp = blueprint_library.find(f"sensor.camera.{cam_config['type']}")
        cam_bp.set_attribute('image_size_x', str(cam_config['width']))
        cam_bp.set_attribute('image_size_y', str(cam_config['height']))
        cam_bp.set_attribute('fov', str(cam_config['fov']))
        cam_transform = carla.Transform(
            carla.Location(x=cam_config['position']['x'], y=cam_config['position']['y'], z=cam_config['position']['z']),
            carla.Rotation(pitch=cam_config['rotation']['pitch'], yaw=cam_config['rotation']['yaw'], roll=cam_config['rotation']['roll'])
        )
        cam_sensor = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actor_list.append(cam_sensor)
        cam_sensor.listen(lambda image, cam_type=cam_config['type']: save_image(image, cam_type, save_dir))

    for lidar_config in config['sensors']['lidar']:
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(lidar_config['channels']))
        lidar_bp.set_attribute('range', str(lidar_config['range']))
        lidar_bp.set_attribute('points_per_second', str(lidar_config['points_per_second']))
        lidar_bp.set_attribute('rotation_frequency', str(lidar_config['rotation_frequency']))
        lidar_bp.set_attribute('upper_fov', str(lidar_config['upper_fov']))
        lidar_bp.set_attribute('lower_fov', str(lidar_config['lower_fov']))
        lidar_bp.set_attribute('horizontal_fov', str(lidar_config['horizontal_fov']))
        lidar_transform = carla.Transform(
            carla.Location(x=lidar_config['position']['x'], y=lidar_config['position']['y'], z=lidar_config['position']['z']),
            carla.Rotation(pitch=lidar_config['rotation']['pitch'], yaw=lidar_config['rotation']['yaw'], roll=lidar_config['rotation']['roll'])
        )
        lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        actor_list.append(lidar_sensor)
        lidar_sensor.listen(lambda data: process_lidar_data(data, save_dir))

    return actor_list

def main():
    SAVE_DIR = "data"
    CONFIG_PATH = "sim_config.yaml"
    CLIENT_NAME = os.environ.get("CLIENT_NAME", "localhost")
    SPAWN_POINT_INDEX = 15
    MAP_NAME = 'Town02'

    # Load sensor configuration
    config = load_sensor_config(CONFIG_PATH)

    # Delete all files inside images/
    os.system(f"rm -rf {SAVE_DIR}/*")

    # Connect to the CARLA server
    client = carla.Client(CLIENT_NAME, 2000)
    client.set_timeout(10.0)
    print("Client version:", client.get_client_version())
    client.load_world(MAP_NAME)
    world = client.get_world()

    # Set synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    spawn_points = world.get_map().get_spawn_points()
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[SPAWN_POINT_INDEX])
    spawn_points.pop(SPAWN_POINT_INDEX)

    actor_list = [vehicle]
    try:
        actor_list.extend(spawn_sensors(world, blueprint_library, vehicle, config, SAVE_DIR))
        actor_list.extend(spawn_npcs(world, blueprint_library, config, spawn_points))
        print("Sensors spawned and listening...")

        # Simulation loop
        while True:


            # Get the camera matrix 
            # world_2_camera = np.array(camera_sensor.get_transform().get_inverse_matrix())
            bboxes_info = []
            print("Getting image...")
            try:
                image = image_queue.get(block=False)
            except queue.Empty:
                world.tick()
                continue

            for obj in actor_list:
                # Filter out the ego vehicle
                if obj.id == vehicle.id:
                    continue
                bb = obj.bounding_box

                dist = obj.get_transform().location.distance(vehicle.get_transform().location)

                # Filter for the vehicles within 50m
                if dist >= 50:
                    continue


                # Save the bbox for the lidar
                bbox_npy = utils.get_actor_bb_info(obj, vehicle.get_transform())
                bboxes_info.append(bbox_npy)

            #     # # Calculate the dot product between the forward vector
            #     # # of the vehicle and the vector between the vehicle
            #     # # and the other vehicle. We threshold this dot product
            #     # # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
            #     # forward_vec = vehicle.get_transform().get_forward_vector()
            #     # ray = obj.get_transform().location - vehicle.get_transform().location

            #     # if forward_vec.dot(ray) > 1:
            #     #     verts = [v for v in bb.get_world_vertices(obj.get_transform())]
            #     #     for edge in utils.BBOX_EDGES:
            #     #         p1 = utils.get_image_point(verts[edge[0]], K, world_2_camera)
            #     #         p2 = utils.get_image_point(verts[edge[1]], K, world_2_camera)

            #     #         def clamp(value, min_value, max_value):
            #     #             return max(min_value, min(max_value, value))

            #     #         # Clamp points to the image boundaries
            #     #         p1_clamped = (clamp(p1[0], 0, CAMERA_WIDTH-1), clamp(p1[1], 0, CAMERA_HEIGHT-1))
            #     #         p2_clamped = (clamp(p2[0], 0, CAMERA_WIDTH-1), clamp(p2[1], 0, CAMERA_HEIGHT-1))

            #     #         # Draw the line only if at least one original point is within the image boundaries
            #     #         if ((0 <= p1[0] < CAMERA_WIDTH and 0 <= p1[1] < CAMERA_HEIGHT) or
            #     #             (0 <= p2[0] < CAMERA_WIDTH and 0 <= p2[1] < CAMERA_HEIGHT)):
            #     #             cv2.line(img, (int(p1_clamped[0]),int(p1_clamped[1])), (int(p2_clamped[0]),int(p2_clamped[1])), (255,0,0, 255), 1)

            bboxes_npy = np.array(bboxes_info)
            bboxes_npy.reshape(-1, 9)
            np.save(f"{SAVE_DIR}/{'%d'%image.frame}_bbox.npy", bboxes_npy)
            # cv2.imwrite(f"{SAVE_DIR}/{'%06d'%image.frame}_bbox.png", img)

            world.tick()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Destroying actors...")
        for actor in actor_list:
            actor.destroy()

        print("Done.")

if __name__ == "__main__":
    main()
