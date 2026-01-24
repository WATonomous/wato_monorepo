import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from typing import Any, Optional

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor

from lanelet_msgs.srv import GetRoute, GetCorridor
from lanelet_msgs.msg import Lanelet, Corridor

from local_planning.lattice_planner import (
    PlannerConverter,
    PlannerCostCalculator,
    PlannerSampler,
    VehicleState,
    LatticePlanner,
)


#TODO register as an component
class PlanningNode(LifecycleNode):
    def __init__(self, node_name = "planning_node"):
        super().__init__(node_name)
        # Declare topics 
        self.declare_parameter(
            "odom_topic",
            "/ego/odom",
            ParameterDescriptor(
                description="Odometry topic to receive ego car information"
            ),
        )

        self.declare_parameter(
            "lanelet_namespace",
            "/world_modeling",
            ParameterDescriptor(
                description="namespace for lanelet service nodes"
            ),
        )

        self.declare_parameter(
            "service_timeout",
            1.0,
            ParameterDescriptor(
                description="service status timeout for lanelet services"
            ),
        )

        # Vehicle Data
        self.car_pose: Optional[PoseStamped] = None
        self.odom_recieved = None

        # Lane Data
        self.lanelet_corridor_cli: Optional[Any] = None
        self.lanelet_corridor_req: Optional[Any] = None
        self.lanelet_corridor = None
        self.lanelet_corridor_fut = None

        # Timers 
        self.corridor_update_timer_: Optional[object] = None

        # Input Subscribers
        self.odom_subscriber_: Optional[object] = None
        self.lanelet_corridor_subscriber_ = None
        self.lanelet_routing_graph_subscriber_ = None
        self.lanelet_costmap_subscriber_ = None

        # Path Publisher
        self.path_publisher_ = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        
        # Needed Services
        self.service_timeout = float(self.get_parameter("service_timeout").value)

        lanelet_namespace =  self.get_parameter("lanelet_namespace").value
        lanelet_corridor_srv_name = "/get_corridor"

        self.lanelet_corridor_cli = self.create_client(GetCorridor, lanelet_namespace+lanelet_corridor_srv_name) 
        self.lanelet_corridor_req = GetCorridor.Request()
        self.lanelet_corridor = Corridor()

        self.corridor_update_timer_ = self.create_timer(2, self.corridor_update)
        # self.corridor_update_timer_.cancel()

        self.odom_recieved = False
        self.car_pose = PoseStamped()

        # TODO Create all subscriptions needed at time of configure

        odom_topic = self.get_parameter("odom_topic").value
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        self.lanelet_corridor_subscriber_ = None
        self.lanelet_routing_graph_subscriber_ = None
        self.lanelet_costmap_subscriber_ = None    

        self.path_publisher_ = self.create_lifecycle_publisher(Path, "/action/local_planning/Path", 10)

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        #start timer and actions
        self.corridor_update_timer_.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        # Add anything needed for Deactivating
        self.corridor_update_timer_.cancel()
        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
                
        if self.path_publisher_ is not None:
            self.destroy_lifecycle_publisher(self.path_publisher_)
            self.path_publisher_ = None

        if self.corridor_update_timer_ is not None:
            self.destroy_timer(self.corridor_update_timer_)
            self.corridor_update_timer_ = None

        if self.odom_subscriber_ is not None:
            self.destroy_subscription(self.odom_subscriber_)
            self.odom_subscriber_ = None  # important: avoid double-destroy later

        if self.lanelet_corridor_cli is not None:
            self.destroy_client(self.lanelet_corridor_cli)
            self.lanelet_corridor_cli = None
            self.lanelet_corridor_req = None
            self.lanelet_corridor_fut = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self.path_publisher_ is not None:
            self.destroy_lifecycle_publisher(self.path_publisher_)
            self.path_publisher_ = None

        if self.corridor_update_timer_ is not None:
            self.destroy_timer(self.corridor_update_timer_)
            self.corridor_update_timer_ = None

        if self.odom_subscriber_ is not None:
            self.destroy_subscription(self.odom_subscriber_)
            self.odom_subscriber_ = None  # important: avoid double-destroy later

        if self.lanelet_corridor_cli is not None:
            self.destroy_client(self.lanelet_corridor_cli)
            self.lanelet_corridor_cli = None
            self.lanelet_corridor_req = None
            self.lanelet_corridor_fut = None

        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS
    
    def odom_callback(self, msg):
        self.car_pose.pose = msg.pose.pose
        self.car_pose.header = msg.header
        self.odom_recieved = True
    
    def request_lanelet_corridor(self):
        self.lanelet_corridor_req.from_lanelet_id = 0
        self.lanelet_corridor_req.to_lanelet_id = 0
        # Maximum corridor length (meters). Use 0 for full route.
        self.lanelet_corridor_req.max_length_m = 0
        # Recommended: 0.5m for lattice planners
        self.lanelet_corridor_req.sample_spacing_m = 0.5
        # e.g., 1 = include one lane left and one lane right (if they exist)
        self.lanelet_corridor_req.num_lanes_each_side = 1

        if not self.lanelet_corridor_cli.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().warn("get_corridor service not available")
            return None
            
        future = self.lanelet_corridor_cli.call_async(self.lanelet_corridor_req)
        
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()
        return future

    def corridor_update(self):
        # TODO Make async checking better than this
        if self.lanelet_corridor_fut is None:
            self.lanelet_corridor_fut = self.request_lanelet_corridor()
            return

        if not self.lanelet_corridor_fut.done():
            return

        self.lanelet_corridor = self.lanelet_corridor_fut.result()
        self.lanelet_corridor_fut = None

        # TODO Process corridor
        


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()