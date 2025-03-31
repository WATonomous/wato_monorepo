import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from model_predictive_path_integral.mppi_core import mppi_core
from model_predictive_path_integral.helper import euler_from_quaternion

class MPPINode(Node):
    def __init__(self):
        super.__init__("MPPINode") #run ROS2 node init

        self.mppi_core = mppi_core() 
        print("Set-up MPPI node")

        #subscribe to get vehicle odometry
        self.state_odom_subscription = self.create_subscription(
            Odometry, #message type
            '/carla/ego/odometry', #message name
            self.state_odom_callback,#function to be called with (msg) 
            10 #number of messages to be queued at a time
        )

        #subscribe to get vehicle state
        self.vehicle_state_subscription = self.create_subscription(
            CarlaEgoVehicleStatus,
            '/carla/ego/vehicle_status',
            self.vehicle_state_callback,
            10
        )

        #publish final control command
        self.control_publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego/vehicle_control_cmd',
            10
        )
        #publish goal to carla
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/carla/ego/goal', 10)
        
        # Subscribe to waypoints from CARLA
        self.waypoints_subscription = self.create_subscription(
            Path, '/carla/ego/waypoints', self.waypoints_callback, 10)
        
        #create timer to send commands
        self.timer = self.create_timer(0.5,
            self.timer_callback)
        
        print("pub/sub init")
    def vehicle_state_callback(self,msg):
        self.mppi_core.v0 = msg.Velocity
        Quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w]
        _, _, self.mppi_core.theta0 = euler_from_quaternion(Quaternion)
    def state_odom_callback(self,msg):
        self.mppi_core.x0 = msg.pose.pose.position.x
        self.mppi_core.y0 = msg.pose.pose.position.y

    def waypoints_callback(self,msg):
        pass
    def goal_publisher(self,x,y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal_msg)
        



def main(args=None): #allows functions to accept optional arguments
    rclpy.init(args=args)#args = args allows ros2 to process command line arguments
    mppi_node = MPPINode()
    rclpy.spin()
    mppi_node.destroy_node()
    rclpy.shutdown()

        

