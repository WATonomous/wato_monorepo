import rclpy
from rclpy.node import Node

from action_msgs import ControlCommands, VehicleState, WaypointArray 
from carla_core import CarlaCore  # Import the CARLA core logic


class CarlaNode(Node):
    def __init__(self):
        super().__init__('CarlaNode')

        # Initialize CARLA core object (manages CARLA setup and running)
        self.carla_core = CarlaCore(self.publish_state)

        # Publishers
        self.vehicle_state_publisher = self.create_publisher(VehicleState, '/carla/vehicle_state', 10)
        self.waypoints_publisher = self.create_publisher(WaypointArray, '/carla/waypoints', 10)

        # Subscribers
        self.control_subscription = self.create_subscription(
            ControlCommands, '/mpc/control_commands', self.control_callback, 10)


        self.carla_core.start_main_loop()


    def publish_state(self):
        # Get the current vehicle state from carla_core
        vehicle_state = self.carla_core.get_vehicle_state()
        if vehicle_state:
            # Publish the vehicle state to MPC node
            state_msg = state_msgs()
            state_msg.pos_x = vehicle_state['x']
            state_msg.pos_y = vehicle_state['y']
            state_msg.angle = vehicle_state['theta']
            state_msg.velocity = vehicle_state['velocity']
            self.vehicle_state_publisher.publish(state_msg)

    def publish_waypoints(self):
        # Get the current waypoints from carla_core
        waypoints = self.carla_core.get_waypoints()
        if waypoints:
            # Create an instance of WaypointArray message
            waypoint_array_msg = WaypointArray()

            for wp in waypoints:
                # Create a Waypoint message for each waypoint in the list
                waypoint_msg = Waypoint()
                waypoint_msg.x = wp[0]  # x-coordinate
                waypoint_msg.y = wp[1]  # y-coordinate

                # Append each Waypoint message to the WaypointArray message
                waypoint_array_msg.waypoints.append(waypoint_msg)

            # Publish the WaypointArray message
            self.waypoints_publisher.publish(waypoint_array_msg)

    def control_callback(self, msg):
        """
        This function will be called when a control message is received from the MPC Node.
        It will forward the control commands to the carla_core.
        """
        # Extract steering and throttle from the message
        steering_angle = msg.steering_angle
        throttle = msg.throttle

        # Send control commands to CARLA via carla_core
        self.carla_core.apply_control(steering_angle, throttle)


def main(args=None):
    rclpy.init(args=args)
    carla_node = CarlaNode()
    rclpy.spin(carla_node)
    carla_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    