import carla
import casadi as ca
import numpy as np
import datetime
import os
import shutil

from boxconstraint import BoxConstraint

SIM_DURATION = 500  # Simulation duration in time steps in Carla

## SETUP ##
# Connect to CARLA
client = carla.Client('localhost', 2000)
maps = [m.replace('/Game/Carla/Maps/', '') for m in client.get_available_maps()]
print('Available maps: ', maps)
world = client.get_world()
mymap = world.get_map()
print('Using map: ', mymap.name)
spectator = world.get_spectator()

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

# Function to move the spectator camera
def move_spectator_to_vehicle(vehicle, spectator, distance=10):
    vehicle_location = vehicle.get_location()
    # Set viewing angle to slightly above the vehicle
    spectator_transform = carla.Transform(vehicle_location + carla.Location(z=distance), carla.Rotation(pitch=-90))
    spectator.set_transform(spectator_transform)


# Use recommended spawn points
spawn_points = mymap.get_spawn_points()
spawn_point = spawn_points[0]

# Spawn vehicle
vehicles = world.get_actors().filter('vehicle.*')
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


def generate_waypoint_relative_to_spawn(forward_offset=0, sideways_offset=0):
    waypoint_x = spawn_point.location.x + spawn_point.get_forward_vector().x * forward_offset + spawn_point.get_right_vector().x * sideways_offset
    waypoint_y = spawn_point.location.y + spawn_point.get_forward_vector().y * forward_offset + spawn_point.get_right_vector().y * sideways_offset
    return ca.vertcat(waypoint_x, waypoint_y)


def generate_waypoint(x, y):
    return ca.vertcat(x, y)


waypoints = []

for i in range(SIM_DURATION):
    waypoints.append(generate_waypoint_relative_to_spawn(-10, 0))

# Parameters
params = {
    'L': 2.875  # Wheelbase of the vehicle. Source : https://www.tesla.com/ownersmanual/model3/en_us/GUID-56562137-FC31-4110-A13C-9A9FC6657BF0.html
}
T = 2.0  # Prediction horizon in seconds
N = int(T / TIME_STEP)  # Prediction horizon in time steps
dt = TIME_STEP  # Time step for discretization
state_dim = 4  # Dimension of the state [x, y, theta, v]
control_dim = 2  # Dimension of the control input [steering angle, acceleration]

# Initialize Opti object
opti = ca.Opti()

# Declare variables
X = opti.variable(state_dim, N + 1)  # state trajectory variables over prediction horizon
U = opti.variable(control_dim, N)  # control trajectory variables over prediction horizon
P = opti.parameter(state_dim)  # initial state parameter
Q_base = ca.MX.eye(state_dim)  # Base state penalty matrix (emphasizes position states)
weight_increase_factor = 1.00  # Increase factor for each step in the prediction horizon
R = ca.MX.eye(control_dim)  # control penalty matrix for objective function
W = opti.parameter(2, N)  # Reference trajectory parameter

# Objective
obj = 0
for k in range(N):
    Q = Q_base * (weight_increase_factor ** k)  # Increase weight for each step in the prediction horizon

    x_k = X[:, k]  # Current state
    u_k = U[:, k]  # Current control input
    x_next = X[:, k + 1]  # Next state

    x_ref = ca.vertcat(W[:, k],
                       ca.MX.zeros(state_dim - 2, 1))  # Reference state with waypoint and zero for other states

    dx = x_k - x_ref  # Deviation of state from reference state
    du = u_k  # Control input deviation (assuming a desired control input of zero)

    # Quadratic cost with reference state and control input
    obj += ca.mtimes([ca.mtimes(dx.T, Q), dx]) + ca.mtimes(
        [ca.mtimes(du.T, R), du])  # Minimize quadratic cost and deviation from reference state

opti.minimize(obj)

# Maximum steerin angle for dynamics
max_steering_angle_deg = max(wheel.max_steer_angle for wheel in
                             vehicle.get_physics_control().wheels)  # Maximum steering angle in degrees (from vehicle physics control
max_steering_angle_rad = max_steering_angle_deg * (ca.pi / 180)  # Maximum steering angle in radians

# Dynamics (Euler discretization using bicycle model)
for k in range(N):
    steering_angle_rad = U[0, k] * max_steering_angle_rad  # Convert normalized steering angle to radians

    opti.subject_to(X[:, k + 1] == X[:, k] + dt * ca.vertcat(
        X[3, k] * ca.cos(X[2, k]),
        X[3, k] * ca.sin(X[2, k]),
        (X[3, k] / params['L']) * ca.tan(steering_angle_rad),
        U[1, k]
    ))

# Constraints
opti.subject_to(X[:, 0] == P)  # Initial state constraint

# Input constraints
steering_angle_bounds = [-1.0, 1.0]
acceleration_bounds = [-1.0, 1.0]
lb = np.array([steering_angle_bounds[0], acceleration_bounds[0]]).reshape(-1, 1)
ub = np.array([steering_angle_bounds[1], acceleration_bounds[1]]).reshape(-1, 1)
action_space = BoxConstraint(lb=lb, ub=ub)

# State constraints
# x_bounds = [-10000, 1000]  # x position bounds (effectively no bounds)
# y_bounds = [-1000, 1000]  # y position bounds (effectively no bounds)
# theta_bounds = [0, 360]  # theta bounds in degrees
# v_bounds = [-10, 10]  # velocity bounds
# lb = np.array([x_bounds[0], y_bounds[0], theta_bounds[0], v_bounds[0]]).reshape(-1, 1)
# ub = np.array([x_bounds[1], y_bounds[1], theta_bounds[1], v_bounds[1]]).reshape(-1, 1)
# state_space = BoxConstraint(lb=lb, ub=ub)

# Apply constraints to optimization problem
for i in range(N):
    # Input constraints
    opti.subject_to(action_space.H_np @ U[:, i] <= action_space.b_np)

    # State constraints
    # opti.subject_to(state_space.H_np @ X[:, i] <= state_space.b_np)

# Setup solver
acceptable_dual_inf_tol = 1e11
acceptable_compl_inf_tol = 1e-3
acceptable_iter = 15
acceptable_constr_viol_tol = 1e-3
acceptable_tol = 1e-6

opts = {"ipopt.acceptable_tol": acceptable_tol,
        "ipopt.acceptable_constr_viol_tol": acceptable_constr_viol_tol,
        "ipopt.acceptable_dual_inf_tol": acceptable_dual_inf_tol,
        "ipopt.acceptable_iter": acceptable_iter,
        "ipopt.acceptable_compl_inf_tol": acceptable_compl_inf_tol,
        "ipopt.hessian_approximation": "limited-memory",
        "ipopt.print_level": 0}
opti.solver('ipopt', opts)

# Variables for main loop below
# Array to store closed-loop trajectory states (X and Y coordinates)
closed_loop_data = []
open_loop_data = []
residuals_data = []

# Initialize warm-start parameters
prev_sol_x = None
prev_sol_u = None

# Main Loop
for i in range(SIM_DURATION - N):  # Subtract N since we need to be able to predict N steps into the future
    print("Iteration: ", i)

    move_spectator_to_vehicle(vehicle, spectator)

    # Draw current waypoints in CARLA
    for waypoint in waypoints[i:i + N]:
        waypoint_x = float(np.array(waypoint[0]))
        waypoint_y = float(np.array(waypoint[1]))

        carla_waypoint = carla.Location(x=waypoint_x, y=waypoint_y, z=0.5)

        extent = carla.Location(x=0.5, y=0.5, z=0.5)
        world.debug.draw_box(box=carla.BoundingBox(carla_waypoint, extent * 1e-2),
                             rotation=carla.Rotation(pitch=0, yaw=0, roll=0), life_time=TIME_STEP * 10, thickness=0.5,
                             color=carla.Color(255, 0, 0))

    #  Fetch initial state from CARLA
    x0 = vehicle.get_transform().location.x
    y0 = vehicle.get_transform().location.y
    theta0 = vehicle.get_transform().rotation.yaw / 180 * ca.pi
    velocity_vector = vehicle.get_velocity()
    v0 = ca.sqrt(velocity_vector.x ** 2 + velocity_vector.y ** 2)

    print("Current x: ", x0)
    print("Current y: ", y0)
    print("Current yaw: ", vehicle.get_transform().rotation.yaw)
    print("Current theta: ", theta0)
    print("Current velocity: ", v0)

    # Store current state in the closed-loop trajectory data
    if i > 0:
        closed_loop_data.append([x0, y0, theta0, v0])

    # Set initial state for optimization problem
    initial_state = ca.vertcat(x0, y0, theta0, v0)
    opti.set_value(P, initial_state)

    # Set the reference trajectory for the current iteration
    opti.set_value(W, ca.horzcat(*waypoints[i:i + N]))  # Concatenate waypoints

    if prev_sol_x is not None and prev_sol_u is not None:
        # Warm-starting the solver with the previous solution
        opti.set_initial(X, prev_sol_x)
        opti.set_initial(U, prev_sol_u)

    # Solve the optimization problem
    sol = opti.solve()

    # If the solver is successful, apply the first control input to the vehicle
    if sol.stats()['success']:
        u = sol.value(U[:, 0])

        # Bound acceleration and steering angle to [-1, 1]
        u[0] = np.clip(u[0], -1, 1)
        u[1] = np.clip(u[1], -1, 1)

        print("Steering angle: ", u[0])
        print("Acceleration: ", u[1])

        if u[1] < 0:
            vehicle.apply_control(carla.VehicleControl(throttle=-u[1], steer=u[0], reverse=True))
        else:
            vehicle.apply_control(carla.VehicleControl(throttle=u[1], steer=u[0]))

        # Store open-loop trajectory data with control input applied to vehicle
        open_loop_trajectory = sol.value(X)
        open_loop_trajectory = open_loop_trajectory.T.reshape(-1, state_dim)
        open_loop_trajectory = np.hstack((open_loop_trajectory, np.tile(u, (N + 1, 1))))
        open_loop_data.append(open_loop_trajectory)

        # Compute and store residuals if i > 0 since we need a previous state to compare
        if i > 0:
            predicted_state = prev_sol_x[:, 1]  # Predicted next state from the previous solution
            actual_state = np.array([x0, y0, theta0, v0])  # Current actual state from CARLA
            residual = actual_state - predicted_state
            residuals_data.append(residual)

        # Update previous solution variables for warm-starting next iteration
        prev_sol_x = sol.value(X)
        prev_sol_u = sol.value(U)

    else:
        print("Error in optimization problem.")

    print("")
    world.tick()  # Tick the CARLA world
