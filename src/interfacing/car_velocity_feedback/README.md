# Car Velocity Feedback

**Package:** `car_velocity_feedback`

This package provides a node that estimates the vehicle's longitudinal body velocity ($v_{body}$) based on the wheel speeds of the front wheels and the current steering angle.

## Algorithm

The node utilizes the **Ackermann Bicycle Model** approximation. It simplifies the 4-wheel vehicle into a 2-wheel bicycle model by averaging the front wheels.

### Calculation Steps

1. **Average Front Wheel Speed:**
   We take the average of the front-left ($v_{FL}$) and front-right ($v_{FR}$) wheel speeds to approximate the velocity of the single front wheel in the bicycle model ($v_{front}$).

   $$
   v_{front} = \frac{v_{FL} + v_{FR}}{2}
   $$

   *Note: The input wheel speeds from `oscc_interfacing` are in **km/h**, so we convert them to mean **m/s** before calculation.*

2. **Project to Body Frame:**
   Using the current steering angle ($\delta$), we project the front wheel velocity onto the vehicle's longitudinal axis to find the body velocity ($v_{body}$).

   $$
   v_{body} = v_{front} \times \cos(\delta)
   $$

### Reference

For more details on mobile robot kinematics and the bicycle model, refer to the [ROS 2 Controllers Documentation](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html).

## Topics

| Name | Type | Description |
| :--- | :--- | :--- |
| `body_velocity_feedback` | `std_msgs/Float64` | Estimated body velocity in m/s. |
| `/oscc_interfacing/wheel_speeds` | `roscco_msg/WheelSpeeds` | Input wheel speeds (km/h). |
| `/oscc_interfacing/steering_angle` | `roscco_msg/SteeringAngle` | Input steering angle (radians). |
