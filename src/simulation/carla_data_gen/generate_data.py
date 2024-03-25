import carla
import os
import queue
import numpy as np
import utils 
import cv2

client_name = os.environ.get("CLIENT_NAME", "DOES NOT EXIST")
if client_name == "DOES NOT EXIST":
    raise Exception("The environment variable for the container name of the carla server has not been set")

# Connect to the client and retrieve the world object
client = carla.Client(client_name, 2000)
client.set_timeout(100.0)
client.reload_world()
print(client.get_available_maps())
# map_name = '/Game/Carla/Maps/Town10HD' # Example map name, CARLA supports Town01, Town02, etc.
# client.load_world(map_name)
# Get the world
world = client.get_world()
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
print("Got world")

import time
import random

SAVE_DIR = "sim_images"

weather = {
  "ClearNoon": carla.WeatherParameters.ClearNoon,
  "CloudyNoon": carla.WeatherParameters.CloudyNoon,
  "WetNoon": carla.WeatherParameters.WetNoon,
  "WetNoon": carla.WeatherParameters.WetCloudyNoon,
  "WetNoon": carla.WeatherParameters.MidRainyNoon,
  "HardRainNoon": carla.WeatherParameters.HardRainNoon,
  "SoftRainNoon": carla.WeatherParameters.SoftRainNoon,
  "ClearSunset": carla.WeatherParameters.ClearSunset,
  "CloudySunset": carla.WeatherParameters.CloudySunset,
  "WetSunset": carla.WeatherParameters.WetSunset,
  "WetCloudySunset": carla.WeatherParameters.WetCloudySunset,
  "MidRainSunset": carla.WeatherParameters.MidRainSunset,
  "HardRainSunset": carla.WeatherParameters.HardRainSunset,
  "SoftRainSunset": carla.WeatherParameters.SoftRainSunset
  }

image_queue = queue.Queue()

def save_image(image, image_type):
    """
    Saves the image to a file.
    """
    print("Saving image")
    if image_type == "depth":
        image.save_to_disk('%s/%06d_%s.png' % (SAVE_DIR, image.frame, image_type), carla.ColorConverter.LogarithmicDepth)
    else:
        image.save_to_disk('%s/%06d_%s.png' % (SAVE_DIR, image.frame, image_type))
        image_queue.put(image)

actor_list = []
print("Starting main")
try:

    print("Set settings")
    
    blueprint_library = world.get_blueprint_library()
    
    traffic_manager = client.get_trafficmanager(8000) # Default port for Traffic Manager
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)
    traffic_manager.set_synchronous_mode(True)
    
    # Find the Tesla Model 3 blueprint
    tesla_model_3_bp = blueprint_library.find('vehicle.tesla.model3')
    
    # Choose a spawn point
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0]
    
    # Spawn the Tesla
    car = world.spawn_actor(tesla_model_3_bp, spawn_point)
    
    # Set up the camera blueprint
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    K = utils.build_projection_matrix(800, 600, 120)
    
    blueprint_library = world.get_blueprint_library()
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '800')
    cam_bp.set_attribute('image_size_y', '600')
    cam_bp.set_attribute('fov', '120')
    # cam_bp.set_transform(camera_transform)

    # Set up the depth camera blueprint
    depth_cam_bp = blueprint_library.find('sensor.camera.depth')
    depth_cam_bp.set_attribute('image_size_x', '800')
    depth_cam_bp.set_attribute('image_size_y', '600')
    depth_cam_bp.set_attribute('fov', '120')
    # depth_cam_bp.set_transform(camera_transform)

    # Spawn the camera sensor
    camera_sensor = world.spawn_actor(cam_bp, camera_transform, attach_to=car)
    actor_list.append(camera_sensor)

    # Spawn the depth camera sensor
    depth_camera_sensor = world.spawn_actor(depth_cam_bp, camera_transform, attach_to=car)
    actor_list.append(depth_camera_sensor)
    print("Spawned sensors")
    
    camera_sensor.listen(lambda image: save_image(image, 'camera'))
    depth_camera_sensor.listen(lambda image: save_image(image, 'depth'))

    # Spawn NPC vehicles
    for i in range(1,len(spawn_points)):
        print(i)
        vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
        npc = world.try_spawn_actor(vehicle_bp, spawn_points[i])
        if npc:
            npc.set_autopilot(True)

    # Measure and log FPS
    start_time = time.time()
    frames = 0
    print("Starting simulation")
    car.set_autopilot(True)
    while True:
        world.tick()
        frames += 1
        elapsed_time = time.time() - start_time
        print(frames)

        image = image_queue.get()

        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

        # Fetch all actors
        all_actors = {}
        all_actors["vehicles"] = world.get_actors().filter('*vehicle*')
        all_actors["pedestrians"] = world.get_actors().filter('*walker*')
        all_actors["traffic"] = world.get_actors().filter('*traffic*')

        # Get the camera matrix 
        world_2_camera = np.array(camera_sensor.get_transform().get_inverse_matrix())
        bboxes = []

        for actor_list in all_actors.values():
            print("actor list len is ", len(actor_list))
            for obj in actor_list:

                # Filter out the ego vehicle
                # if npc.id == car.id:
                #     continue
                bb = obj.bounding_box

                dist = obj.get_transform().location.distance(car.get_transform().location)

                # Filter for the vehicles within 50m
                if dist >= 50:
                    continue
                print("dist is", dist)
                # Calculate the dot product between the forward vector
                # of the vehicle and the vector between the vehicle
                # and the other vehicle. We threshold this dot product
                # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                forward_vec = car.get_transform().get_forward_vector()
                ray = obj.get_transform().location - car.get_transform().location

                if forward_vec.dot(ray) > 1:
                    p1 = utils.get_image_point(bb.location, K, world_2_camera)
                    verts = [v for v in bb.get_world_vertices(obj.get_transform())]
                    for edge in utils.BBOX_EDGES:
                        p1 = utils.get_image_point(verts[edge[0]], K, world_2_camera)
                        p2 = utils.get_image_point(verts[edge[1]],  K, world_2_camera)
                        cv2.line(img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), (255,0,0, 255), 1)        

        cv2.imwrite(f"{SAVE_DIR}/bbox_{image.frame}.png", img)
        time.sleep(0.1)

except Exception as e:
    # This will catch any exception that occurs within the try block
    print(f'An error occurred: {e}')
finally:
    
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
