import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import math
import numpy as np
from local_planning.lattice_planner import PlannerConverter, PlannerCostCalculator, PlannerSampler, VehicleState, LatticePlanner


class LocalPlanningNode(Node):
    ASSUMED_LANE_WIDTH = 3.5  # default lane width when no estimate is created

    def __init__(self):
        super().__init__('local_planning_node')

        self.query_frq = 1.0

        self.query_timer = self.create_timer(
            self.query_frq, self.publish_query_point)  # publish point every second
        self.car_timer = self.create_timer(1.0, self.car_publish_marker)

        self.start_time = self.get_clock().now()

        # Car position
        # TODO: replace car position with subscriber to car position
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_z = 0.0
        self.angle = -np.pi
        self.vel_x = 0
        self.vel_y = 0

        self.lane_scope = 6

        # Lattice settings:
        self.horizon = 30  # (m)
        self.lane_subdiv = 3  # number of latteral "columns" in the lattice
        self.lane_width = 0

        self.lattices = []
        self.current_trajectory = []
        self.traj_idx = 0
        self.curr_marker = None

        self.conv = PlannerConverter(self.lane_to_coord)
        init_state = VehicleState(self.car_x, self.car_y, self.angle, 0)
        traj_cost = PlannerCostCalculator()
        sample = PlannerSampler()
        self.planner = LatticePlanner(init_state, self._lane_info_,
                                      self.conv, traj_cost, sample, lookAheadDist=self.horizon)

        self.lanlet_subscriber_ = self.create_subscription(
            MarkerArray,  # Message type
            'hd_map_current_lane',  # Topic name
            self.marker_callback,  # Callback function
            10  # QoS profile (queue size)
        )

        self.global_path_subscriber_ = self.create_subscription(
            MarkerArray,  # Message type
            'hd_map_route',  # Topic name
            self.path_callback,  # Callback function
            10  # QoS profile (queue size)
        )
        self.lattice_publisher_ = self.create_publisher(Marker, 'lattice_marker', 10)
        self.trajectory_publisher_ = self.create_publisher(Marker, 'trajectory_marker', 10)
        self.car_publisher_ = self.create_publisher(Marker, 'car_marker', 10)
        self.query_point_publisher_ = self.create_publisher(PointStamped, 'query_point', 10)

    def _lane_info_(self):
        return self.lane_width * self.lane_subdiv, self.lane_subdiv

    def findNearestOnRoad(self):
        # TODO: make this a smart search
        # Search points close to where the car last was
        # and update the current node based on that

        minDist = float('Infinity')
        for i in range(len(self.roads)):
            for j in range(len(self.roads[i])):
                vec, start, points = self.roads[i][j]
                mid = points[len(points)//2]
                mid_vec = np.array([mid.x, mid.y])-start
                for k in range(len(points)):

                    pos = np.array([self.car_x, self.car_y])
                    point = np.array([points[k].x, points[k].y])

                    dist = np.linalg.norm(pos - point)

                    if dist < minDist:
                        minDist = dist
                        self.curr_marker = (i, j, k)

        self.create_lattices()

    def run_planner(self):
        # Find target lane
        target_lane = 0
        if self.curr_marker is not None:
            road, lane, s_p = self.curr_marker
            target_lane = -(len(self.roads[road])-1-lane)
            if len(self.roads[road]) > 1:
                vec1, pt1, _ = self.roads[road][0]
                _, pt2, _ = self.roads[road][1]

                # unknown if the next lane is on the right or left
                side_vec = pt2 - pt1

                # vector pointing left to lane
                perp_vec = np.empty_like(vec1)
                perp_vec[0] = -vec1[1]
                perp_vec[1] = vec1[0]

                unit_side_vec = side_vec / np.linalg.norm(side_vec)
                unit_perp = perp_vec / np.linalg.norm(perp_vec)

                # if they are in opposite directions (i.e next lane is on the right)
                # then the sum of the unit vectors should result in a vector with
                # norm less than 1
                if np.linalg.norm(unit_perp + unit_side_vec) < 0.5:
                    target_lane *= -1
        self.get_logger().info(f'{target_lane}')
        # run planner
        res = self.planner.run(target_lane)

        # Check planner was able to generate a plan
        if res is not None:
            # Place lattice nodes used for the plan
            # into a list
            parent = res.incomingState
            route = [res]
            while parent is not None:
                route.append(parent)
                parent = parent.incomingState

            # Extract trajectory connecting lattice nodes
            self.current_trajectory = []
            self.traj_idx = 0
            for node in route[::-1]:
                # pos_x, pos_y, theta, curvature
                count = 0
                for x, y, t, k in node.incomingTrajectory:

                    count += 1
                    if count % 10 == 0:
                        self.current_trajectory.append([x, y, t, k])

    def car_publish_marker(self):
        self.run_planner()
        traj_marker = Marker()
        traj_marker.header = Header()
        traj_marker.header.frame_id = 'map'
        traj_marker.type = Marker.POINTS  # Set marker type to POINTS
        traj_marker.action = Marker.ADD

        # Set the scale of the points (size in meters)
        traj_marker.scale.x = 1.0  # Width of each point
        traj_marker.scale.y = 1.0  # Height of each point

        # Set the color for all points (RGBA)
        traj_marker.color.r = 0.0
        traj_marker.color.g = 0.0
        traj_marker.color.b = 1.0
        traj_marker.color.a = 1.0  # Fully opaque

        # Convert your array of points to ROS Point messages
        traj_marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in self.current_trajectory]
        self.trajectory_publisher_.publish(traj_marker)

        # make car follow path
        if (len(self.current_trajectory) > 0 and self.curr_marker):

            self.car_x = self.current_trajectory[self.traj_idx][0]
            self.car_y = self.current_trajectory[self.traj_idx][1]
            self.angle = self.current_trajectory[self.traj_idx][2]
            new_state = VehicleState(self.car_x, self.car_y, self.angle,
                                     self.current_trajectory[self.traj_idx][3], (0, 0))
            self.planner.setNewState(new_state)
            if self.traj_idx < len(self.current_trajectory)-1:
                self.traj_idx += 1
            self.get_logger().info(f'{self.car_x, self.car_y, self.angle}')

            self.findNearestOnRoad()

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

    def interpolateRoad(self, road: tuple, s, start_index: int):
        _, start, points = road

        final_point = start
        points = np.array([[p.x, p.y] for p in points])

        dxdt = np.gradient(points[:, 0])
        dydt = np.gradient(points[:, 1])
        dvdt = np.array([[dxdt[i], dydt[i]] for i in range(dxdt.size)])
        dsdt = np.sqrt(dxdt * dxdt + dydt * dydt)
        tangent = np.array([1/dsdt] * 2).transpose() * dvdt
        deriv_tangent_x = np.gradient(tangent[:, 0])
        deriv_tangent_y = np.gradient(tangent[:, 1])
        dTdt = np.array([[deriv_tangent_x[i], deriv_tangent_y[i]]
                        for i in range(deriv_tangent_x.size)])
        len_dTdt = np.sqrt(deriv_tangent_x * deriv_tangent_x + deriv_tangent_y * deriv_tangent_y)
        normal = np.array([1/len_dTdt]*2).transpose() * dTdt
        d2sdt2 = np.gradient(dsdt)
        d2xdt2 = np.gradient(dxdt)
        d2ydt2 = np.gradient(dydt)

        curvature = np.abs(d2xdt2 * dydt - dxdt * d2ydt2) / (dxdt * dxdt + dydt * dydt)**1.5

        pt_curvature = curvature[0]
        pt_normal = normal[0]
        pt_tangent = tangent[0]
        for i in range(start_index, len(points)-1):
            pt1 = points[i]
            pt2 = points[i+1]

            vec = pt2-pt1
            dist = np.linalg.norm(vec)

            if dist > s:
                break

            s -= dist
            final_point = pt2
            pt_curvature = curvature[i+1]
            pt_normal = curvature[i+1]
            pt_tangent = tangent[i+1]

        return final_point, s, pt_curvature, pt_normal, pt_tangent

    def lane_to_coord(self, s: float, l: float):
        if self.curr_marker is None:
            return None
        road, lane, s_p = self.curr_marker

        pt = None
        curvature = None
        normal = None
        tangent = None
        while s > 0 and road + 1 <= len(self.roads):
            curr_road = self.roads[road][lane]
            final_pt = curr_road[2][-1]
            final_pt = np.array([final_pt.x, final_pt.y])
            pt, s, curvature, normal, tangent = self.interpolateRoad(curr_road, s, s_p)
            remaining_dist = np.linalg.norm(final_pt - pt)
            if s > remaining_dist and lane == len(self.roads[road])-1:
                road += 1
                lane = 0
                s_p = 0
            else:
                break

        perp_vec = np.empty_like(tangent)
        perp_vec[0] = -tangent[1]
        perp_vec[1] = tangent[0]

        pt += perp_vec * l
        theta = np.arctan2(tangent[1], tangent[0])

        kappa = curvature

        return pt[0], pt[1], theta, kappa

    def path_callback(self, msg):
        # Process the received message
        minDist = float('Infinity')
        min_idx = None

        self.roads = []

        self.curr_marker = None

        prev_vec = None
        prev_start = None
        lane = 0
        for i in range(len(msg.markers)):
            start_point = np.array([msg.markers[i].points[0].x, msg.markers[i].points[0].y])
            mid_point = np.array([msg.markers[i].points[len(msg.markers[i].points)//2].x,
                                 msg.markers[i].points[len(msg.markers[i].points)//2].y])
            end_point = np.array([msg.markers[i].points[-1].x, msg.markers[i].points[-1].y])

            curr_vec = end_point - start_point

            if prev_vec is not None:
                # Check lane is parallel
                # TODO: needs fixing to consider identifying if two curved paths are parallel
                if np.linalg.norm(np.cross(prev_vec, curr_vec)) / (np.linalg.norm(prev_vec) * np.linalg.norm(curr_vec)) < 0.1:
                    # Calculate perp. distance (seperatation distance between lanes)
                    p = start_point - prev_start
                    sep_dist = np.linalg.norm(np.cross(prev_vec/np.linalg.norm(prev_vec), p))

                    # double check this is an adjacent lane
                    if self.lane_width == 0:
                        self.lane_width = self.ASSUMED_LANE_WIDTH  # temp if not set

                    if (sep_dist < self.lane_width * 1.3 and sep_dist > self.lane_width * 0.7):
                        self.roads[len(self.roads)-1].append((curr_vec,
                                                              start_point, msg.markers[i].points))
                        lane += 1
                    else:
                        self.roads.append([(curr_vec, start_point, msg.markers[i].points)])
                        lane = 0
                else:
                    self.roads.append([(curr_vec, start_point, msg.markers[i].points)])
                    lane = 0

                prev_vec = curr_vec
                prev_start = start_point

            else:
                self.roads.append([(curr_vec, start_point, msg.markers[i].points)])
                prev_vec = curr_vec
                prev_start = start_point

            mid_vec = mid_point-start_point
            if np.linalg.norm(np.cross(curr_vec, mid_vec)) / (np.linalg.norm(curr_vec) * np.linalg.norm(mid_vec)) >= 0.1:
                continue

            for j in range(len(msg.markers[i].points)):
                pos = np.array([self.car_x, self.car_y])
                point = np.array([msg.markers[i].points[j].x, msg.markers[i].points[j].y])

                dist = np.linalg.norm(pos - point)

                if dist < minDist:
                    minDist = dist
                    self.curr_marker = (len(self.roads)-1, lane, j)

        self.get_logger().info(f'Received message with {len(msg.markers)} markers')
        self.create_lattices()

    def marker_callback(self, msg):
        # estimating a constant lane width
        sqr_x_dist = (msg.markers[0].points[0].x-msg.markers[1].points[0].x) ** 2
        sqr_y_dist = (msg.markers[0].points[0].y-msg.markers[1].points[0].y) ** 2
        dist = (sqr_x_dist + sqr_y_dist) ** (1/2)
        self.lane_width = dist

    def create_lattices(self):
        # TODO: Retrieve this directly from planner instead
        self.lattices = []
        for s in range(10, self.horizon+1, 10):
            for l in range(-1, 2):
                l *= self.lane_width
                coord = self.lane_to_coord(s, l)
                self.lattices.append(self.lane_to_coord(s, l))

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
        marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in self.lattices]

        # Publish the marker
        if len(self.lattices) > 0:
            self.lattice_publisher_.publish(marker)

    def publish_query_point(self):
        # use to move car

        point_msg = PointStamped()
        point_msg.header.frame_id = "map"  # Set the frame of reference
        point_msg.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
        point_msg.point.x = self.car_x  # Example x-coordinate
        point_msg.point.y = self.car_y  # Example y-coordinate
        point_msg.point.z = self.car_z  # Example z-coordinate

        # Publish the message
        self.query_point_publisher_.publish(point_msg)
        self.get_logger().info(
            f'Publishing PointStamped: {point_msg.point.x}, {point_msg.point.y}, {point_msg.point.z}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
