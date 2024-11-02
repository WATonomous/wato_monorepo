import carla
import numpy as np
import datetime
import os
import shutil

SIM_DURATION = 500  # Simulation duration in time steps in Carla
TIME_STEP = 0.05
PREDICTION_HORIZON = 2.0 

class CarlaCore:
    def __init__(self, publish_state):
        self.publish_state = publish_state

        self.spectator = None
        self.vehicles = None
        self.spawn_point = None

        self.setup_carla()

        self.vehicle_state = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'velocity': 0.0,
            'iteration': 0
        }

        self.waypoints = []

    def setup_carla(self):
        ## SETUP ##
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        maps = [m.replace('/Game/Carla/Maps/', '') for m in client.get_available_maps()]
        print('Available maps: ', maps)
        world = client.get_world()
        mymap = world.get_map()
        print('Using map: ', mymap.name)
        self.spectator = world.get_spectator()

        # CARLA Settings
        settings = world.get_settings()
        # Timing settings
        settings.synchronous_mode = True  # Enables synchronous mode
        TIME_STEP = 0.05  # Time step for synchronous mode
        settings.fixed_delta_seconds = TIME_STEP
        # Physics substep settings
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10

        world.apply_settings(settings)

        # Output client and world objects to console
        print(client)
        print(world)

        # Use recommended spawn points
        self.spawn_points = mymap.get_spawn_points()
        self.spawn_point = spawn_points[0]

        # Spawn vehicle
        self.vehicles = world.get_actors().filter('vehicle.*')
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]
        print("Vehicle blueprint attributes:")
        for attr in vehicle_bp:
            print('  - {}'.format(attr))

        if len(vehicles) == 0:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        else:
            # Reset world
            for vehicle in vehicles:
                vehicle.destroy()
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(vehicle)


    def get_waypoints(self):
        """Generate and return the list of waypoints relative to the vehicle's spawn point."""

        for i in range(SIM_DURATION): 
            self.waypoints.append(self.generate_waypoint_relative_to_spawn(-10, 0))

        return self.waypoints

    def generate_waypoint_relative_to_spawn(self, forward_offset=0, sideways_offset=0):
        waypoint_x = spawn_point.location.x + spawn_point.get_forward_vector().x * forward_offset + spawn_point.get_right_vector().x * sideways_offset
        waypoint_y = spawn_point.location.y + spawn_point.get_forward_vector().y * forward_offset + spawn_point.get_right_vector().y * sideways_offset
        return [waypoint_x, waypoint_y]

    def get_vehicle_state(self):
        """
        Retrieves the current state of the vehicle in the CARLA world.
        The state includes position, orientation (theta), velocity, and iteration count.
        """

        #  Fetch initial state from CARLA
        x0 = vehicle.get_transform().location.x
        y0 = vehicle.get_transform().location.y
        theta0 = vehicle.get_transform().rotation.yaw / 180 * ca.pi
        velocity_vector = vehicle.get_velocity()
        v0 = ca.sqrt(velocity_vector.x ** 2 + velocity_vector.y ** 2)

        # Update vehicle state
        self.vehicle_state['x'] = x0
        self.vehicle_state['y'] = y0
        self.vehicle_state['theta'] = theta0
        self.vehicle_state['velocity'] = v0

        return self.vehicle_state

    def apply_control(self, steering_angle, throttle):
        """
        Applies the received control commands to the vehicle.
        """
        if throttle < 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=-throttle, steer=steering_angle, reverse=True))
        else:
            self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steering_angle))

    def start_main_loop(self):
        """
        Main loop to continuously publish vehicle states and waypoints.
        """
        N = int(PREDICTION_HORIZON/TIME_STEP)

        for i in range(SIM_DURATION - N):  # Subtract N since we need to be able to predict N steps into the future
            print("Iteration: ", i)

            self.vehicle_state['iteration'] = i

            move_spectator_to_vehicle()

            # Draw current waypoints in CARLA
            for waypoint in self.waypoints[i:i + N]:
                waypoint_x = float(np.array(waypoint[0]))
                waypoint_y = float(np.array(waypoint[1]))

                carla_waypoint = carla.Location(x=waypoint_x, y=waypoint_y, z=0.5)

                extent = carla.Location(x=0.5, y=0.5, z=0.5)
                world.debug.draw_box(box=carla.BoundingBox(carla_waypoint, extent * 1e-2),
                                    rotation=carla.Rotation(pitch=0, yaw=0, roll=0), life_time=TIME_STEP * 10, thickness=0.5,
                                    color=carla.Color(255, 0, 0))
            
            self.publish_state() # MPC Should Start after this !!!!
            
            # Should automatically apply control after publishing state to mpc

            print("")
            world.tick()  # Tick the CARLA world
            
    
    def move_spectator_to_vehicle(distance=10):
        vehicle_location = vehicle.get_location()
        # Set viewing angle to slightly above the vehicle
        spectator_transform = carla.Transform(vehicle_location + carla.Location(z=distance), carla.Rotation(pitch=-90))
        spectator.set_transform(spectator_transform)

    def publish_state(self):
        """To be overridden by CarlaNode for publishing the vehicle state."""
        pass
