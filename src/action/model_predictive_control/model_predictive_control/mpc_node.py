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

import rclpy 
from rclpy.node import Node

from action_msgs import ControlCommands, VehicleState, WaypointArray 
from model_predictive_control.mpc_core import MPCCore


class MPCNode(Node):
    def __init__(self):
        super().__init__('MPCNode')

        self.mpc_core = MPCCore()

        # Subscribe to vehicle state
        self.state_subscription = self.create_subscription(
            VehicleState, '/carla/vehicle_state', self.vehicle_state_callback, 10)

        # Subscribe to waypoints from CARLA
        self.waypoints_subscription = self.create_subscription(
            WaypointArray, '/carla/waypoints', self.waypoints_callback, 10)
            
        self.control_publisher = self.create_publisher(ControlCommands, '/mpc/control_commands', 10)

    def vehicle_state_callback(self, msg):
        steering_angle, throttle = self.mpc_core.compute_control(msg)

        control_msg = control_msgs()
        control_msg.steering_angle = steering_angle
        control_msg.throttle = throttle
        self.control_publisher.publish(control_msg)

    def waypoints_callback(self, msg):
        self.mpc_core.update_waypoints(msg)


def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPCNode()
    rclpy.spin(mpc_node)
    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()