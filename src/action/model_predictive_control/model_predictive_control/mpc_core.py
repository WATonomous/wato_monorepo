import casadi as ca
import numpy as np
import datetime
import os
import shutil

from model_predictive_control.boxconstraint import BoxConstraint

TIME_STEP = 0.05
PREDICTION_HORIZON = 2.0
SIM_DURATION = 500

class MPCCore:
    def __init__(self):

        self.params = {
            'L': 2.875  # Wheelbase of the vehicle. Source : https://www.tesla.com/ownersmanual/model3/en_us/GUID-56562137-FC31-4110-A13C-9A9FC6657BF0.html
        }
        self.T = PREDICTION_HORIZON  # Prediction horizon in seconds
        self.N = int(self.T / TIME_STEP)  # Prediction horizon in time steps
        self.dt = TIME_STEP  # Time step for discretization
        self.state_dim = 4  # Dimension of the state [x, y, theta, v]
        # Dimension of the control input [steering angle, acceleration]
        self.control_dim = 2

        # Initialize Opti object
        self.opti = ca.Opti()

        # Declare variables
        # state trajectory variables over prediction horizon
        self.X = self.opti.variable(self.state_dim, self.N + 1)
        # control trajectory variables over prediction horizon
        self.U = self.opti.variable(self.control_dim, self.N)
        self.P = self.opti.parameter(self.state_dim)  # initial state parameter
        # Base state penalty matrix (emphasizes position states)
        self.Q_base = ca.MX.eye(self.state_dim)
        # Increase factor for each step in the prediction horizon
        self.weight_increase_factor = 1.00
        # control penalty matrix for objective function
        self.R = ca.MX.eye(self.control_dim)
        # Reference trajectory parameter
        self.W = self.opti.parameter(2, self.N)

        # Objective
        self.obj = 0

        self.raw_waypoints = []
        self.waypoints = []

        self.closed_loop_data = []
        self.open_loop_data = []
        self.residuals_data = []

        self.prev_sol_x = None
        self.prev_sol_u = None

        # MPC Initial Setup
        # self.get_waypoints()
        self.setup_mpc()
        self.setup_constraints()
        self.setup_solver()

        # Vehicle States
        self.x0 = 0
        self.y0 = 0
        self.theta0 = 0
        self.v0 = 0

    def convert_waypoints(self):
        """
        Convert raw waypoints (alternating x, y in a flat list) to CasADi-compatible waypoints.
        """
        self.waypoints = []  # Clear old waypoints

        if len(self.raw_waypoints) % 2 != 0:  # Check if raw_waypoints is even
            print(
                "Error: raw_waypoints length is not even. Ignoring the last unpaired value.")
            self.raw_waypoints = self.raw_waypoints[:-1]

        for i in range(0, len(self.raw_waypoints), 2):
            x, y = self.raw_waypoints[i], self.raw_waypoints[i + 1]
            waypoint = self.generate_waypoint(x, y)
            self.waypoints.append(waypoint)

    def generate_waypoint(x, y):  # Convert to CasADi format and add to the waypoints list
        return ca.vertcat(x, y)

    def setup_mpc(self):
        """
        Setup the MPC problem with CasADi
        """

        for k in range(self.N):
            # Increase weight for each step in the prediction horizon
            Q = self.Q_base * (self.weight_increase_factor ** k)

            x_k = self.X[:, k]  # Current state
            u_k = self.U[:, k]  # Current control input
            x_next = self.X[:, k + 1]  # Next state

            # Reference state with waypoint and zero for other states
            x_ref = ca.vertcat(
                self.W[:, k], ca.MX.zeros(self.state_dim - 2, 1))

            dx = x_k - x_ref  # Deviation of state from reference state
            # Control input deviation (assuming a desired control input of
            # zero)
            du = u_k

            # Quadratic cost with reference state and control input
            # Minimize quadratic cost and deviation from reference state
            self.obj += ca.mtimes([ca.mtimes(dx.T, Q), dx]) + \
                ca.mtimes([ca.mtimes(du.T, self.R), du])

        self.opti.minimize(self.obj)

        # Maximum steerin angle for dynamics
        # self.max_steering_angle_deg = max(wheel.max_steer_angle for wheel in vehicle.get_physics_control(
        # ).wheels)  # Maximum steering angle in degrees (from vehicle physics control
        self.max_steering_angle_deg = 30.0

        self.max_steering_angle_rad = self.max_steering_angle_deg * (ca.pi / 180)  # Maximum steering angle in radians

        # Dynamics (Euler discretization using bicycle model)
        for k in range(self.N):
            # Convert normalized steering angle to radians
            steering_angle_rad = self.U[0, k] * self.max_steering_angle_rad

            self.opti.subject_to(self.X[:, k +
                                        1] == self.X[:, k] +
                                 self.dt *
                                 ca.vertcat(self.X[3, k] *
                                            ca.cos(self.X[2, k]), self.X[3, k] *
                                            ca.sin(self.X[2, k]), (self.X[3, k] /
                                                                   self.params['L']) *
                                            ca.tan(steering_angle_rad), self.U[1, k]))

    def setup_constraints(self):

        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.P)

        # Input constraints
        steering_angle_bounds = [-1.0, 1.0]
        acceleration_bounds = [-1.0, 1.0]
        lb = np.array([steering_angle_bounds[0],
                      acceleration_bounds[0]]).reshape(-1, 1)
        ub = np.array([steering_angle_bounds[1],
                      acceleration_bounds[1]]).reshape(-1, 1)
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
        for i in range(self.N):
            # Input constraints
            self.opti.subject_to(action_space.H_np @
                                 self.U[:, i] <= action_space.b_np)

            # State constraints
            # opti.subject_to(state_space.H_np @ X[:, i] <= state_space.b_np)

    def setup_solver(self):
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
        self.opti.solver('ipopt', opts)

    def compute_control(self, i):
        """
        Update the vehicle state based on the incoming ROS message.
        :param vehicle_state: VehicleState message with current position, velocity, and angle.
        """
        # Update P (initial state) with the new vehicle state
        self.opti.set_value(self.P, ca.vertcat(x, y, theta0, v0))

        print("Current x: ", x0)
        print("Current y: ", y0)
        print("Current theta: ", theta0)
        print("Current velocity: ", v0)

        if i > 0:
            # Original Code need i > 0
            self.closed_loop_data.append([x0, y0, theta0, v0])

        initial_state = ca.vertcat(x0, y0, theta0, v0)
        self.opti.set_value(self.P, initial_state)

        # Set the reference trajectory for the current iteration
        self.opti.set_value(self.W, ca.horzcat(
            *self.waypoints[i:i + self.N]))  # Concatenate waypoints

        if self.prev_sol_x is not None and self.prev_sol_u is not None:
            # Warm-starting the solver with the previous solution
            self.opti.set_initial(self.X, self.prev_sol_x)
            self.opti.set_initial(self.U, self.prev_sol_u)

        """
        Solve the MPC optimization problem and return the control commands.
        :return: Tuple of (steering angle, throttle).
        """
        steering_angle = 0
        throttle = 0

        # Solve the optimization problem
        sol = self.opti.solve()

        if sol.stats()['success']:
            # Extract control inputs (steering angle, throttle)
            u = sol.value(self.U[:, 0])
            steering_angle = np.clip(u[0], -1.0, 1.0)
            throttle = np.clip(u[1], -1.0, 1.0)

            print("Steering angle: ", u[0])
            print("Acceleration: ", u[1])

            # Store open-loop trajectory data with control input applied to
            # vehicle
            open_loop_trajectory = sol.value(self.X)
            open_loop_trajectory = open_loop_trajectory.T.reshape(
                -1, self.state_dim)
            open_loop_trajectory = np.hstack(
                (open_loop_trajectory, np.tile(u, (N + 1, 1))))
            self.open_loop_data.append(open_loop_trajectory)

            if i > 0:
                # Predicted next state from the previous solution
                predicted_state = self.prev_sol_x[:, 1]
                # Current actual state from CARLA
                actual_state = np.array([x0, y0, theta0, v0])
                residual = actual_state - predicted_state
                self.residuals_data.append(residual)

            # Update previous solution variables for warm-starting next
            # iteration
            self.prev_sol_x = sol.value(self.X)
            self.prev_sol_u = sol.value(self.U)

        else:
            print("Error in optimization problem.")

        return steering_angle, throttle
