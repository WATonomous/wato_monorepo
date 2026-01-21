# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import numpy as np
from abc import ABC, abstractmethod
import copy


class Converter(ABC):
    @abstractmethod
    def convertToState(self, vertex: tuple) -> list:
        pass


class PlannerConverter(Converter):
    def __init__(self, toStateFnc):
        self.toState = toStateFnc

    def convertToState(self, vertex):
        return self.toState(vertex[0], vertex[1])


class CostCalculator(ABC):
    @abstractmethod
    def calculateTrajectoryCost(
        self,
        trajectory,
        sample,
        preferred_lane=0,
        obstacle_positions=[],
        max_curvature_rate=0.1,
        comfort_weight=1.0,
        obstacle_weight=5.0,
        curvature_weight=2.0,
        lane_weight=1.0,
    ):
        pass


class PlannerCostCalculator(CostCalculator):
    def calculateTrajectoryCost(
        self,
        trajectory,
        sample,
        preferred_lane=0,
        obstacle_positions=[],
        max_curvature_rate=0.1,
        comfort_weight=1,
        obstacle_weight=5,
        curvature_weight=2,
        lane_weight=10,
    ):
        """
        Calculate the cost of a trajectory based on obstacle avoidance, physical limitations,
        passenger comfort, and behavioral preferences.

        Parameters:
            trajectory (list): List of points along the trajectory [(x, y, theta, kappa), ...]
            preferred_lane (float): The y-coordinate representing the desired lane center.
            obstacle_positions (list): List of obstacle positions [(x_obs, y_obs), ...].
            max_curvature_rate (float): Maximum allowable rate of change of curvature.
            comfort_weight (float): Weight for lateral acceleration cost.
            obstacle_weight (float): Weight for obstacle avoidance cost.
            curvature_weight (float): Weight for curvature rate cost.
            lane_weight (float): Weight for lane preference cost.

        Returns:
            float: Total cost of the trajectory.
        """
        # TODO(wato): base this on cost map as well (for collisions / obstacles)
        total_cost = 0.0
        previous_kappa = None

        for i, (x, y, theta, kappa) in enumerate(trajectory):
            # Obstacle avoidance cost
            obstacle_cost = 0.0
            for x_obs, y_obs in obstacle_positions:
                distance = np.hypot(x - x_obs, y - y_obs)
                if distance > 0:  # Avoid division by zero
                    if distance < 1:
                        return False
                    obstacle_cost += 1 / distance  # Higher cost for closer obstacles
            total_cost += obstacle_weight * obstacle_cost

            # Curvature rate change cost (physical limitation)
            if previous_kappa is not None:
                curvature_rate = abs(kappa - previous_kappa)
                if curvature_rate > max_curvature_rate:
                    total_cost += curvature_weight * (
                        curvature_rate - max_curvature_rate
                    )

            # Lateral acceleration cost (passenger comfort)
            lateral_acceleration = (
                kappa  # Assuming lateral acceleration is proportional to curvature
            )
            total_cost += comfort_weight * abs(lateral_acceleration)

            # Lane preference cost
            lane_cost = abs(
                sample[1] - preferred_lane
            )  # Minimize distance to preferred lane
            total_cost += lane_weight * lane_cost

            previous_kappa = kappa  # Update previous kappa for the next iteration

        # Add trajectory-dependent costs based on vehicle performance
        for i in range(1, len(trajectory)):
            x_prev, y_prev, theta_prev, kappa_prev = trajectory[i - 1]
            x, y, theta, kappa = trajectory[i]

            # Calculate instantaneous velocity and acceleration between points
            ds = np.hypot(x - x_prev, y - y_prev)
            dt = 1.0  # Assume time step of 1 for simplicity, can be adjusted as needed
            velocity = ds / dt
            acceleration = (velocity - (np.hypot(x_prev - x, y_prev - y) / dt)) / dt

            # Velocity and acceleration costs (can be adjusted with weights if needed)
            if acceleration > 0:  # Optional: penalize positive acceleration
                total_cost += comfort_weight * acceleration
            elif acceleration < 0:  # Optional: penalize deceleration
                total_cost += comfort_weight * abs(acceleration)
        return total_cost


class Sampler(ABC):
    @abstractmethod
    def generateSamples(self, lane_info):
        pass


class PlannerSampler(Sampler):
    def generateSamples(self, lane_info):
        samples = []
        max_sample_bucket = lane_info["horizon"][1]

        for sample_b in range(1, max_sample_bucket + 1):
            sample = sample_b * lane_info["horizon"][2]
            for lane_bucket in range(-1, 2):
                lane = lane_bucket * lane_info["lane"][2]
                samples.append((sample, lane))
        return samples


class VehicleState:
    def __init__(self, x, y, theta, kappa, vertex=None):
        self.state = [x, y, theta, kappa]
        self.vertex = vertex

        self.incomingState = None
        self.cost = np.inf
        self.incomingTrajectory = []

    def getState(self):
        return self.state

    def getVertex(self) -> tuple:
        return self.vertex

    def setIncoming(self, incomingState: "VehicleState", trajectory: list):
        self.incomingState = incomingState
        self.incomingTrajectory = trajectory

    def setCost(self, cost):
        if self.incomingState and not np.isinf(self.incomingState.getCost()):
            cost += self.incomingState.getCost()
        self.cost = cost

    def getCost(self):
        return self.cost


class LatticePlanner:
    HORIZON_STEP_SIZE = 10  # (m)

    def __init__(
        self,
        startState: "VehicleState",
        getLaneInfo,
        converter: "Converter",
        trajectoryCostCalculator: "CostCalculator",
        sampler: "Sampler",
        lookAheadDist: float = 30,
        followingTargetState=False,
        targetState: "VehicleState" = None,
    ):
        self.followingTargetState = followingTargetState
        self.targetState = targetState

        self.trajectoryCostCalculator = trajectoryCostCalculator
        self.sampler = sampler

        self.s_horizon = lookAheadDist
        self.s_buckets = int(self.s_horizon / self.HORIZON_STEP_SIZE)
        self.s_step_size = self.s_horizon / self.s_buckets

        self.laneInfo = getLaneInfo
        self.l_width, self.l_buckets = (
            self.laneInfo()
        )  # TODO(wato): should call "l_width" road with
        self.l_step_size = self.l_width / self.l_buckets

        self.arcinfo = {
            "horizon": [self.s_horizon, self.s_buckets, self.s_step_size],
            "lane": [self.l_width, self.l_buckets, self.l_step_size],
        }

        self.lattice = [[] for i in range(self.s_buckets + 1)]
        self.lattice[0].append(startState)

        self.converter = converter

    def run(self, lane=0, depth=2):
        self._update_lane_info_()
        for i in range(depth):
            curr_lattice = copy.deepcopy(self.lattice)
            for s_ind in range(self.s_buckets):
                for i in range(len(curr_lattice[s_ind])):
                    curr_state = curr_lattice[s_ind][i]
                    samples = self.sampler.generateSamples(self.arcinfo)

                    for sample in samples:
                        s, _ = sample
                        if s <= s_ind * self.s_step_size:
                            continue
                        start = curr_state.getState()
                        target = self.converter.convertToState(sample)
                        if target is None:
                            continue
                        try:
                            new_state_vec, trajectory = self.generate_trajectory(
                                start, target
                            )
                        except Exception:
                            continue

                        x, y, t, k = tuple(new_state_vec)
                        new_state = VehicleState(x, y, t, k, sample)
                        new_state.setIncoming(curr_state, trajectory)
                        cost = self.trajectoryCostCalculator.calculateTrajectoryCost(
                            trajectory,
                            sample,
                            lane * -self.l_step_size,
                            [(-15, 0.8796)], #TODO hard coded obstacle
                        )
                        if cost is False:
                            continue
                        new_state.setCost(cost)
                        new_s, _ = new_state.getVertex()
                        s_bucket = new_s // self.s_step_size
                        self.lattice[int(s_bucket)].append(new_state)

        best_path = self._get_sorted_paths_()
        return best_path

    def _update_lane_info_(self):
        self.l_width, self.l_buckets = self.laneInfo()
        self.l_step_size = self.l_width / self.l_buckets

        self.arcinfo = {
            "horizon": [self.s_horizon, self.s_buckets, self.s_step_size],
            "lane": [self.l_width, self.l_buckets, self.l_step_size],
        }

    def _get_sorted_paths_(self):
        # paths = []
        best_cost = np.inf
        best_state = None
        for i in range(len(self.lattice)):
            if len(self.lattice[i]) == 0:
                continue
            for j in range(len(self.lattice[i])):
                state = self.lattice[i][j]
                vertex = state.getVertex()
                if vertex is None:
                    continue
                s, _ = vertex
                state_cost = state.getCost() - 1000 * s

                if state.incomingState is None:
                    continue

                if state_cost < best_cost:
                    best_state = state
                    best_cost = state_cost

                # paths.append(state)

        # paths.sort(key = lambda state: state.getCost() - 10 * state.getVertex()[0])

        return best_state

    def setNewState(self, new_state):
        self.lattice = [[] for i in range(self.s_buckets + 1)]
        self.lattice[0].append(new_state)

        self.l_width, self.l_buckets = self.laneInfo()
        self.l_step_size = self.l_width / self.l_buckets

    def generate_trajectory(self, start, target, max_iterations=10, tolerance=1e-2):
        """
        Generate a trajectory from start to target using a cubic polynomial for curvature.
        This function was generated using GPTv4.

        Parameters:
            start (list): Initial conditions (x, y, theta, kappa)
            target (list): Target conditions (x, y, theta, kappa)
            max_iterations (int): Maximum number of iterations for convergence
            tolerance (float): Tolerance for endpoint convergence

        Returns:
            trajectory (list): List of points along the generated trajectory [(x, y, theta, kappa), ...]
        """
        # TODO(wato): improve trajectory generation

        def normalize_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        def angle_diff(target_angle, current_angle):
            return normalize_angle(target_angle - current_angle)

        x0, y0, theta0, kappa0 = tuple(start)
        targetX, targetY, targetTheta, targetKappa = tuple(target)

        theta0 = normalize_angle(theta0)
        targetTheta = normalize_angle(targetTheta)

        # Improved initial guess for parameters
        initial_arc_length = np.sqrt((targetX - x0) ** 2 + (targetY - y0) ** 2)
        # start with target curvature and estimated arc length
        p = np.array([targetKappa, targetKappa, targetKappa, initial_arc_length])

        def curvature_polynomial(s, p):
            # Define the cubic polynomial for curvature
            a = kappa0
            b = -(11 * kappa0 - 18 * p[0] + 9 * p[1] - 2 * p[2]) / (2 * p[3])
            c = 9 * (2 * kappa0 - 5 * p[0] + 4 * p[1] - p[2]) / (2 * p[3] ** 2)
            d = -9 * (kappa0 - 3 * p[0] + 3 * p[1] - p[2]) / (2 * p[3] ** 3)
            return a + b * s + c * s**2 + d * s**3

        def integrate_trajectory(p):
            # Integrate using the bicycle model for position updates
            s_values = np.linspace(0, p[3], num=100)  # discretize arc length

            theta = theta0
            x, y = x0, y0
            trajectory = []

            for s in s_values:
                kappa = curvature_polynomial(s, p)
                # incremental theta change
                theta = normalize_angle(theta + kappa * (s_values[1] - s_values[0]))
                x += np.cos(theta) * (s_values[1] - s_values[0])
                y += np.sin(theta) * (s_values[1] - s_values[0])
                trajectory.append((x, y, theta, kappa))

            return trajectory[-1], trajectory

        def compute_error(p):
            # Get the final point and compare to desired target
            final_point, _ = integrate_trajectory(p)
            xf, yf, thetaf, kappaf = final_point

            return np.array(
                [
                    xf - targetX,
                    yf - targetY,
                    angle_diff(thetaf, targetTheta),
                    kappaf - targetKappa,
                ]
            )

        # Iterate using Newton's method to adjust parameters for trajectory alignment
        for i in range(max_iterations):
            error = compute_error(p)

            if np.linalg.norm(error) < tolerance:
                return integrate_trajectory(p)

            # Calculate Jacobian numerically for updates
            jacobian = np.zeros((4, 4))
            delta = 1e-6
            for j in range(4):
                p_perturbed = p.copy()
                p_perturbed[j] += delta
                error_perturbed = compute_error(p_perturbed)
                jacobian[:, j] = (error_perturbed - error) / delta

            # Update parameters using Newton step
            dp = np.linalg.solve(jacobian, -error)
            p += dp

        raise Exception("Failed to converge.")
