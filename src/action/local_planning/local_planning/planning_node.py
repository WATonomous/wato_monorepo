import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped

from rcl_interfaces.msg import ParameterDescriptor

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
                description="Odometry topic to publish ego linear velocity"
            ),
        )

        # Vehicle Data
        self.car_pose: Optional[PoseStamped] = None

        self.odom_subscriber_: Optional[object] = None
        self.lanelet_corridor_subscriber_ = None
        self.lanelet_routing_graph_subscriber_ = None
        self.lanelet_costmap_subscriber_ = None

        self.path_publisher_ = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        
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
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        # Add anything needed for Deactivating
        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        
        self.destroy_lifecycle_publisher(self.path_publisher_)
        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_lifecycle_publisher(self.path_publisher_)
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS
    
    def odom_callback(self, msg):
        self.car_pose.pose = msg.pose.pose
        self.car_pose.header = msg.header


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()