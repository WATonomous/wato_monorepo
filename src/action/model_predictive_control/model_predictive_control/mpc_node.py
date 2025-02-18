# Copyright 2023 WATonomous
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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# from path_planning_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from model_predictive_control.helper import euler_from_quaternion
from model_predictive_control.mpc_core import MPCCore

# For extracting theta from w in quaternion
# from tf_transformations import euler_from_quaternion


class MPCNode(Node):
    def __init__(self):
        super().__init__('MPCNode')

        self.mpc_core = MPCCore()
        print("mpc setup")

        # Subscribe to vehicle state (only retrieving velocity)
        self.state_subscription = self.create_subscription(
            CarlaEgoVehicleStatus,
            '/carla/ego/vehicle_status',
            self.vehicle_state_callback,
            10)

        # Subscribe to get vehicle position/orientation (x, y, w)
        self.state_odom_subscription = self.create_subscription(
            Odometry, '/carla/ego/odometry', self.state_odom_callback, 10)

        self.control_publisher = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/ego/vehicle_control_cmd', 10)

        self.goal_publisher = self.create_publisher(
            PoseStamped, '/carla/ego/goal', 10)

        # Subscribe to waypoints from CARLA
        self.waypoints_subscription = self.create_subscription(
            Path, '/carla/ego/waypoints', self.waypoints_callback, 10)
        
        
        
        self.timer = self.create_timer(
            0.5,  # 0.01 seconds = 10 milliseconds = 100 Hz
            self.timer_callback,
        )
        
        
        print("pub/sub init")

        self.goal_set = False
        self.i = 0
        self.goal_point_x = 10.0
        self.goal_point_y = 10.0
        print("goal set")
        self.publish_goal(self.goal_point_x, self.goal_point_y)
        print("goal pub")

    def vehicle_state_callback(self, msg):
        print("veh_state")
        self.mpc_core.v0 = msg.velocity
        # print(self.mpc_core.v0)
        # Extract theta/yaw/orientation of the car in the x-y plane from
        # quaternion
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w]
        # print(quaternion)
        # print(euler_from_quaternion(quaternion))
        _, _, self.mpc_core.theta0 = euler_from_quaternion(quaternion)

    def waypoints_callback(self, msg):
        self.get_logger().info(f"Received {len(msg.poses)} waypoints")
        
        
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.mpc_core.raw_waypoints.append(x)
            self.mpc_core.raw_waypoints.append(y)
            # print("waypoints")
            # print(x)
            # print(y)

        self.mpc_core.convert_waypoints()
        print("waypoints converted")
        self.goal_set = True

    def state_odom_callback(self, msg):
        self.mpc_core.x0 = msg.pose.pose.position.x
        self.mpc_core.y0 = msg.pose.pose.position.y
        print("state_odom")

    def publish_goal(self, x, y):
        goal_msg = PoseStamped()    
        
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
            
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        # goal_msg.orientation.x = 0.0
        # goal_msg.orientation.y = 0.0
        # goal_msg.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal: x={x}, y={y}")
        
        
        
    # def start_main_loop(self):
    #     # Subtract N since we need to be able to predict N steps into the
    #     # future
        
    #     for i in range(self.mpc_core.SIM_DURATION - self.mpc_core.N):
    #         steering_angle, throttle = self.mpc_core.compute_control(i)
    #         # print("steer and throttle")
    #         # print(steering_angle)
    #         # print(throttle)
    #         control_msg = CarlaEgoVehicleControl()
    #         control_msg.steer = steering_angle
    #         control_msg.throttle = throttle
    #         self.control_publisher.publish(control_msg)
    #     pass
    
    def timer_callback(self):
        if self.goal_set == False:
            return
        steering_angle, throttle = self.mpc_core.compute_control(self.i)
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = steering_angle
        control_msg.throttle = throttle
        self.control_publisher.publish(control_msg)
        self.i += 1



def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPCNode()
    rclpy.spin(mpc_node)
    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
