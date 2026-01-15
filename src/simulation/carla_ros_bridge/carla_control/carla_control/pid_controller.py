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
