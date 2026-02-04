# Eve OSCC Interfacing

ROS2 node that interfaces with the physical OSCC boards through OSCC CAN API

This node does these things:

- Subscribes to /roscco

- Publishes to /oscc_interfacing/is_armed (Just a bool, 100HZ)

- Publishes to /oscc_interfacing/wheel_speeds (4 floats, one per wheel)
- Publishes to /oscc_interfacing/steering_wheel_angle (float, degrees, 0 = centered)
  - You can model this with ackermann reference frames and get an odom for
  - speed and angular velocity for localization

- Is a server for the service /oscc_interfacing/arm
  - Attempt to either arm or disarm.
  - SetBool service: true = attempt to arm, false = attempt to disarm
  - Returns success/fail (message empty)
