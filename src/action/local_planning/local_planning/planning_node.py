import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor

from typing import Any, Optional

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from lanelet_msgs.srv import GetRoute, GetCorridor
from lanelet_msgs.msg import Lanelet, Corridor, CurrentLaneContext, Lanelet, CorridorLane

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
            "lane_context_topic",
            "/lane_context",
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
        self.ahead_lanelet_ids = None
        self.lanelet_corridor_cli: Optional[Any] = None
        self.lanelet_corridor_req: Optional[Any] = None
        self.lanelet_corridor = None
        self.lanelet_corridor_fut = None

        # Timers 
        self.corridor_update_timer_: Optional[object] = None

        # Input Subscribers
        self.odom_subscriber_: Optional[object] = None
        self.lane_context_subscriber_: Optional[object] = None
        self.lanelet_corridor_subscriber_ = None
        self.lanelet_routing_graph_subscriber_ = None
        self.lanelet_costmap_subscriber_ = None

        # Path Publisher
        self.path_marker_publisher_ = None
        self.path_publisher_ = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        
        # Needed Services
        self.service_timeout = float(self.get_parameter("service_timeout").value)

        lanelet_corridor_srv_name = "/get_corridor"

        self.lanelet_corridor_cli = self.create_client(GetCorridor, lanelet_corridor_srv_name) 
        self.lanelet_corridor_req = GetCorridor.Request()
        self.lanelet_corridor = Corridor()

        self.corridor_update_timer_ = self.create_timer(2, self.corridor_update)
        # self.corridor_update_timer_.cancel()

        self.odom_recieved = False
        self.car_pose = PoseStamped()

        # TODO Create all subscriptions needed at time of configure

        odom_topic = self.get_parameter("odom_topic").value
        lane_context_topic = self.get_parameter("lane_context_topic").value

        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.lane_context_subscriber_ = self.create_subscription(
            CurrentLaneContext,
            lane_context_topic,
            self.lane_context_callback,
            10
        )

        self.lanelet_corridor_subscriber_ = None
        self.lanelet_routing_graph_subscriber_ = None
        self.lanelet_costmap_subscriber_ = None    

        self.path_publisher_ = self.create_lifecycle_publisher(Path, "/action/local_planning/Path", 10)
        self.path_marker_publisher_ = self.create_lifecycle_publisher(MarkerArray, "/action/local_planning/Path_Markers", 10)

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

    def lane_context_callback(self, msg):
        self.ahead_lanelet_ids = []
        self.ahead_lanelet_ids.append(msg.current_lanelet.id)
        self.get_logger().info(f"current lanelet id: {msg.current_lanelet.successor_ids}")
        self.ahead_lanelet_ids.extend(msg.current_lanelet.successor_ids)
    
    def corridor_marker_publish(self):
        pass

    def request_lanelet_corridor(self):

        if not self.ahead_lanelet_ids or len(self.ahead_lanelet_ids) < 1:
            self.get_logger().warn("no current lane lanelet data available")
            return 
        
        if not self.lanelet_corridor_cli.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().warn("get_corridor service not available")
            return None

        self.get_logger().info(f"From Lanelet: {self.ahead_lanelet_ids[0]} to Lanelet: {self.ahead_lanelet_ids[-1]}")

        self.lanelet_corridor_req.from_lanelet_id = self.ahead_lanelet_ids[0]
        self.lanelet_corridor_req.to_lanelet_id = self.ahead_lanelet_ids[-1]
        self.lanelet_corridor_req.max_length_m = 0.0
        self.lanelet_corridor_req.sample_spacing_m = 0.5
            
        future = self.lanelet_corridor_cli.call_async(self.lanelet_corridor_req)
        
        return future

    def corridor_update(self):
        # TODO Make async checking better than this
        if self.lanelet_corridor_fut is None:
            self.lanelet_corridor_fut = self.request_lanelet_corridor()
            return

        if not self.lanelet_corridor_fut.done():
            return
        
        if not self.lanelet_corridor_fut.result().success:
            self.get_logger().error(f"Lanelet service result has failed: {self.lanelet_corridor_fut.result().error_message}")


        self.lanelet_corridor = self.lanelet_corridor_fut.result()
        self.lanelet_corridor_fut = None

        self.get_logger().info(f"Reference Lane centre line:" f"{len(self.lanelet_corridor.corridor.right_lane.centerline)}")


        # TODO Process corridor
        


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()