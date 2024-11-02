import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import math

class LocalPlanningNode(Node):
    def __init__(self):
        super().__init__('local_planning_node')

        self.query_frq = 1.0

        self.query_timer = self.create_timer(self.query_frq, self.publish_query_point) #publish point every second
        self.car_timer = self.create_timer(1.0, self.car_publish_marker)
        self.start_time = self.get_clock().now()

        #Car position 
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_z = 0.0
        self.angle = 0
        self.vel_x = 0
        self.vel_y = 0

        self.lane_scope = 6
        self.lane_width = 0

        self.scope_idx = 0
        self.current_centre_lane = []

        #make modular
        self.current_lane_idx = 2
        self.num_lanes = 4 
        self.lattice_density = 4
        self.lattices = []


        self.lanlet_subscriber_ = self.create_subscription(
            MarkerArray,  # Message type
            'hd_map_desired_lane',  # Topic name
            self.marker_callback,  # Callback function
            10  # QoS profile (queue size)
        )
        self.lattice_publisher_ = self.create_publisher(Marker, 'lattice_marker', 10)
        self.car_publisher_ = self.create_publisher(Marker, 'car_marker', 10)
        self.query_point_publisher_ = self.create_publisher(PointStamped, 'query_point', 10)

    def euclidean_distance(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def average_point(self, x1, x2):
        return (x1+x2)/2
    
    def car_publish_marker(self):
        #to move car
        #self.car_x -= 0.5
        #to rotate car
        self.angle += math.radians(1)

        #make car follow path
        if(len(self.current_centre_lane) > 0):
            self.car_x = self.current_centre_lane[self.scope_idx+2][0]
            self.car_y = self.current_centre_lane[self.scope_idx+2][1]
            
            #not sure why i have to multiply by a negative but it just works
            self.vel_x = -1*(self.current_centre_lane[self.scope_idx+2][0]-self.car_x) /self.query_frq
            self.vel_y = (self.current_centre_lane[self.scope_idx+2][1]-self.car_y) /self.query_frq


        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = 'map'
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set the position and orientation
        marker.pose.position.x = self.car_x
        marker.pose.position.y = self.car_y
        marker.pose.position.z = self.car_z
        marker.pose.orientation.z = math.sin(self.angle / 2.0)
        marker.pose.orientation.w = math.cos(self.angle / 2.0)

        # Define the size of the rectangle (e.g., 2 meters long, 1 meter wide, 0.5 meter high)
        marker.scale.x = 2.0  # Length
        marker.scale.y = 1.0  # Width
        marker.scale.z = 0.5  # Height (thickness for visualization)

        # Set color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Blue rectangle
        marker.color.a = 1.0  # Fully opaque

        self.publish_lattice_marker()

        self.car_publisher_.publish(marker)
        self.get_logger().info('Publishing Car Marker')


    def marker_callback(self, msg):
        # Process the received message
        min = float('Infinity')
        min_idx = None

        for i in range(len(msg.markers[0].points)):
            x = self.average_point(msg.markers[0].points[i].x,msg.markers[1].points[i].x)
            y = self.average_point(msg.markers[0].points[i].y,msg.markers[1].points[i].y)
            self.current_centre_lane.append([x,y])
            if(min>self.euclidean_distance(x,self.car_x,y,self.car_y)):
                min = self.euclidean_distance(x,self.car_x,y,self.car_y)
                min_idx = i
                #self.get_logger().info(f'closest point {self.current_centre_lane[min_idx]} from {[self.car_x,self.car_y]} with dist {min}')
     
            
        #estimating a constant lane width for the next x scope also assuming the largest diff is the width
        self.lane_width = max(abs(msg.markers[0].points[min_idx].x-msg.markers[1].points[min_idx].x),abs(msg.markers[0].points[min_idx].y-msg.markers[1].points[min_idx].y))
        self.scope_idx = min_idx

        #printing centre lane
        # if(len(self.current_centre_lane) > 0):
        #     if(self.current_centre_lane[0][0] != self.average_point(msg.markers[0].points[i].x,msg.markers[1].points[i].x)):
        #         self.get_logger().info(f'centre lane: {self.current_centre_lane}')

        self.create_lattices()

        self.get_logger().info(f'Received message with {min_idx} markers')

    def create_lattices(self):
        self.lattices = []
        #assuming constant lane width and no curve
        for j in range(1,self.lattice_density+1):    
            for i in range(self.num_lanes):
                self.lattices.append([(math.copysign(self.lane_scope*j,self.vel_x)) + self.car_x-(math.tan(self.angle)*self.lane_width*(i-self.current_lane_idx)),self.car_y+self.lane_width*(i-self.current_lane_idx),0.0])




    def publish_lattice_marker(self):

        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = 'map'
        marker.type = Marker.POINTS  # Set marker type to POINTS
        marker.action = Marker.ADD

        # Set the scale of the points (size in meters)
        marker.scale.x = 1.0  # Width of each point
        marker.scale.y = 1.0  # Height of each point

        # Set the color for all points (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Fully opaque

        # Convert your array of points to ROS Point messages
        marker.points = [Point(x=pt[0], y=pt[1], z=pt[2]) for pt in self.lattices]

        # Publish the marker
        if len(self.lattices) > 0:
            self.lattice_publisher_.publish(marker)

        self.get_logger().info(f'Publishing Points Marker with {self.lattices} points')

    def publish_query_point(self):
        #use to move car    

        point_msg = PointStamped()
        point_msg.header.frame_id = "map"  # Set the frame of reference
        point_msg.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
        point_msg.point.x = self.car_x # Example x-coordinate
        point_msg.point.y = self.car_y # Example y-coordinate
        point_msg.point.z = self.car_z # Example z-coordinate

        # Publish the message
        self.query_point_publisher_.publish(point_msg)
        self.get_logger().info(f'Publishing PointStamped: {point_msg.point.x}, {point_msg.point.y}, {point_msg.point.z}')



def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
