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
"""PID controller for acceleration tracking."""


class PIDController:
    """Simple PID controller implementation."""

    def __init__(self, kp: float, ki: float, kd: float):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID output.

        Args:
            error: Current error value
            dt: Time step since last update

        Returns:
            PID control output
        """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def reset(self):
        """Reset PID state."""
        self.integral = 0.0
        self.prev_error = 0.0

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
