import carla
import time
import csv
import os

# float range helper
def frange(start, stop, step):
    while start <= stop:
        yield start
        start += step

class SimulationHelper:
    # constants
    MIN_FRICTION = 0.5
    MAX_FRICTION = 5
    STEP_FRICTION = 0.5 # total iterations = 4.5/0.5 = 9

    # velocity in m/s
    MIN_VEL_X = 0
    MAX_VEL_X = 25
    STEP_VEL_X = 5 # total its = 25/5 = 5

    MIN_VEL_Y = 0
    MAX_VEL_Y = 25
    STEP_VEL_Y = 5 # total its = 25/5 = 5

    # ang velocity in deg/s
    MIN_ANG_VEL = -30
    MAX_ANG_VEL = 30
    STEP_ANG_VEL = 10 # total its = 60/10 = 10

    MIN_THROTTLE = 0
    MAX_THROTTLE = 1
    STEP_THROTTLE = 0.1 # total its = 1/0.1 = 10

    # note: steering jump of 7 degrees? from max steer of 70deg
    MIN_STEER = -1
    MAX_STEER = 1
    STEP_STEER = 0.1 # total its = 2/0.1 = 20

    MIN_BRAKE = 0
    MAX_BRAKE = 1
    STEP_BRAKE = 0.1 # total its = 1/0.1 = 10

    MIN_GEAR = 0
    MAX_GEAR = 5
    STEP_GEAR = 1 # total gears = 6
    # total iterations to simulate = 9*5*5*10*20*6*(10+10) = 5400000, under brake*throttle = 0 constraint

    MAX_DURATION = 20
    CSV_HEADER = ['timestamp','friction', 'gear', 'steer', 'throttle', 'brake','location.x', 'location.y', 'location.z', 'vel.x', 'vel.y', 'vel.z', 'accel.x', 'accel.y', 'accel.z', 'rotation.roll', 'rotation.pitch', 'rotation.yaw', 'vel.roll', 'vel.pitch', 'vel.yaw']

    _simState = None
    _elapsed = 0

    PARALLELISM = 16
    # state holder for each parallel instance
    _states = [dict() for _ in range(16)]
    _isDone = False

    _world = None
    _vehicle_bp = None

    def __init__(self):
        print("init")
        hostname = os.getenv("CLIENT_NAME", "localhost")
        # set up output folder structure
        for friction in frange(self.MIN_FRICTION, self.MAX_FRICTION, self.STEP_FRICTION):
            if not os.path.exists('data/%.2f' % friction):
                os.makedirs('data/%.2f' % friction)
        # Connect to the server
        client = carla.Client(hostname, 2000)
        client.set_timeout(10.0)
        print(client.get_available_maps())
        world = client.load_world('/Game/Carla/Maps/MyMap')
        print("set world")
        settings = world.get_settings()
        # want to set synchronous mode + fixed timestep to ensure data repeatability
        settings.synchronous_mode = True # Enables synchronous mode
        settings.fixed_delta_seconds = 0.05 # set fixed delta 
        world.apply_settings(settings)
        print("set settings")
        self._world = world
        # set up
        bp_lib = world.get_blueprint_library()
        self._vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2017')

        # generator for simulation state
        self._simState = filter(
            lambda x: x[3] == 0 or x[5] == 0, # filter out states where there is throttle and brake command
            (
                # produce iterator for tuple of (wheels, v_x, v_y, w, throttle, steer, brake, gear)
                ([# values other than friction obtained from default lincoln mkz 2017 vehicle
                    carla.WheelPhysicsControl(tire_friction=friction, damping_rate=0.250000, max_steer_angle=69.999992, radius=35.500000, max_brake_torque=800.000000, max_handbrake_torque=0.000000), 
                    carla.WheelPhysicsControl(tire_friction=friction, damping_rate=0.250000, max_steer_angle=69.999992, radius=35.500000, max_brake_torque=800.000000, max_handbrake_torque=0.000000), 
                    carla.WheelPhysicsControl(tire_friction=friction, damping_rate=0.250000, max_steer_angle=0.000000, radius=35.500000, max_brake_torque=800.000000, max_handbrake_torque=1600.000000), 
                    carla.WheelPhysicsControl(tire_friction=friction, damping_rate=0.250000, max_steer_angle=0.000000, radius=35.500000, max_brake_torque=800.000000, max_handbrake_torque=1600.000000)],
                    vel_x, vel_y, ang_vel, throttle, steer, brake, gear)
                for friction in frange(self.MIN_FRICTION, self.MAX_FRICTION+0.01, self.STEP_FRICTION)
                for gear in frange(self.MIN_GEAR, self.MAX_GEAR+1, self.STEP_GEAR)
                for throttle in frange(self.MIN_THROTTLE, self.MAX_THROTTLE+0.01, self.STEP_THROTTLE)
                for brake in frange(self.MIN_BRAKE, self.MAX_BRAKE+0.01, self.STEP_BRAKE)
                for steer in frange(self.MIN_STEER, self.MAX_STEER+0.01, self.STEP_STEER)
                for ang_vel in frange(self.MIN_ANG_VEL, self.MAX_ANG_VEL+0.01, self.STEP_ANG_VEL)
                for vel_y in frange(self.MIN_VEL_Y, self.MAX_VEL_Y+0.01, self.STEP_VEL_Y)
                for vel_x in frange(self.MIN_VEL_X, self.MAX_VEL_X+0.01, self.STEP_VEL_X)
            )
        )

        self.initialize()


    # state tracker for next inital simulation state
    def nextState(self):
        self._elapsed += 1
        print("on state", self._elapsed)
        return self._simState.__next__()

    def initialize(self):
        for i in range(self.PARALLELISM):
            self.spawn(i, *self.nextState())
            self._states[i]["finished"] = False

    def spawn(self, index, wheels, vel_x, vel_y, angular_vel, throttle, steer, brake, gear):
        friction=wheels[0].tire_friction
        spawn_transform = carla.Transform(carla.Location(z=0.2+index*10), carla.Rotation())
        # spawn the vehicle and populate it with necessary state values
        csvfile = open("data/%.2f/%.2f_%.2f_%.2f_%.2f_%.2f_%.2f_%d.csv" % (friction, vel_x, vel_y, angular_vel, throttle, steer, brake, gear), 'w', newline='')
        csvfile.write(",".join(self.CSV_HEADER))
        csvfile.write('\n')
        self._states[index]["file"] = csvfile
        vehicle = self._world.spawn_actor(self._vehicle_bp, spawn_transform)
        self._states[index]["vehicle"] = vehicle
        self._states[index]["ready"] = False
        self._states[index]["duration"] = 0
        self._states[index]["metadata"] = (throttle, steer, brake, gear, friction)
        self._states[index]["last_pos"] = 0

        # Set the vehicle's wheel friction coefficients
        physics_control = vehicle.get_physics_control()
        physics_control.wheels = wheels
        vehicle.apply_physics_control(physics_control)


        # set up initial conditions and steering
        #vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, gear=gear))
        #vehicle.set_target_velocity(carla.Vector3D(vel_x, vel_y, 0))
        #vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, angular_vel))

        # store target starting state to be applied when vehicle touches the ground
        self._states[index]["target"] = (carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, gear=gear), carla.Vector3D(vel_x, vel_y, 0), carla.Vector3D(0, 0, angular_vel))

    def _record(self, idx):
        # 'timestamp', 'friction', 'gear', 'steer', 'throttle', 'brake', location, vel, accel, rotation, ang_vel
        timestamp = self._world.get_snapshot().timestamp.elapsed_seconds
        transform = self._states[idx]["vehicle"].get_transform()
        location = transform.location
        rotation = transform.rotation
        l_x, l_y, l_z = location.x, location.y, location.z
        r_r, r_p, r_y = rotation.roll, rotation.pitch, rotation.yaw
        velocity = self._states[idx]["vehicle"].get_velocity()
        v_x, v_y, v_z = velocity.x, velocity.y, velocity.z
        accel = self._states[idx]["vehicle"].get_acceleration()
        a_x, a_y, a_z = accel.x, accel.y, accel.z
        ang_vel = self._states[idx]["vehicle"].get_angular_velocity()
        w_x, w_y, w_z = ang_vel.x, ang_vel.y, ang_vel.z
        friction, gear, steer, throttle, brake = self._states[idx]["metadata"]
        line = [str(timestamp), str(friction), str(gear), str(steer), str(throttle), str(brake), str(l_x), str(l_y), str(l_z), str(v_x), str(v_y), str(v_z), str(a_x), str(a_y), str(a_z), str(r_r), str(r_p), str(r_y), str(w_x), str(w_y), str(w_z)]
        self._states[idx]["file"].write(','.join(line))
        self._states[idx]["file"].write('\n')
        # print(idx, line)

    def tick(self):
        self._world.tick()
        for i in range(self.PARALLELISM):
            if(self._states[i]["ready"]):
                self._record(i)
                self._states[i]["duration"] += 1
            else:
                # logic to wait for vehicle to land after spawning
                if self._states[i]["vehicle"].get_transform().location.z == self._states[i]["last_pos"]:
                    # apply target initial state
                    control, v, w = self._states[i]["target"]
                    self._states[i]["vehicle"].apply_control(control)
                    self._states[i]["vehicle"].set_target_velocity(v)
                    self._states[i]["vehicle"].set_target_angular_velocity(w)
                    self._states[i]["ready"] = True
                else:
                    # update last position of vehicle
                    self._states[i]["last_pos"] = self._states[i]["vehicle"].get_transform().location.z

            # check if vehicle duration exceeds max duration, if so delete and respawn a new vehicle
            if self._states[i]["duration"] >= self.MAX_DURATION:
                self._states[i]["vehicle"].destroy()
                self._states[i]["file"].close()
                state = None
                try:
                    self.spawn(i, *self.nextState())
                except:
                    # if reached max iterations mark current index as complete
                    self._states[i]["finished"] = True
        if all(x["finished"] for x in self._states):
            self._isDone = True


    def test(self):
        while(True):
            print(self.nextState())

                    # writer.writerow({'timestamp': current_time, 'throttle': throttle, 'steer': steer, 'brake': brake, 'gear': gear, 'location.x': vehicle_transform.location.x, 'location.y': vehicle_transform.location.y, 'location.z': vehicle_transform.location.z, 'rotation.roll': vehicle_transform.rotation.roll, 'rotation.pitch': vehicle_transform.rotation.pitch, 'rotation.yaw': vehicle_transform.rotation.yaw, 'latitude': sensor_data['gnss'][0], 'longitude': sensor_data['gnss'][1], 'altitude': sensor_data['gnss'][2], 'gyro.x': sensor_data['imu']['gyro'].x, 'gyro.y': sensor_data['imu']['gyro'].y, 'gyro.z': sensor_data['imu']['gyro'].z, 'accel.x': sensor_data['imu']['accel'].x, 'accel.y': sensor_data['imu']['accel'].y, 'accel.z': sensor_data['imu']['accel'].z, 'compass': sensor_data['imu']['compass']})


test = SimulationHelper()

while not test._isDone:
    test.tick()