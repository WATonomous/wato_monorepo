#ifndef WORLD_MODELING_HD_MAP_ROUTER_
#define WORLD_MODELING_HD_MAP_ROUTER_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "common_msgs/msg/obstacle.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Exceptions.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include "stop_sign_reg_elem.hpp"
#include "pedestrian_reg_elem.hpp"
#include "stop_sign_reg_elem.hpp"

lanelet::GPSPoint ros_gps_msg_to_lanelet_gps_point(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

class HDMapRouter {
  public:
    HDMapRouter();

    bool set_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr);

    bool set_projector(std::shared_ptr<lanelet::Projector> projector);

    lanelet::LaneletMapPtr get_lanelet();

    lanelet::BasicPoint3d project_gps_to_point3d(lanelet::GPSPoint gps_point);
    lanelet::GPSPoint project_point3d_to_gps(lanelet::BasicPoint3d point3d);

    lanelet::ConstLanelet get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point);
    
    // TODO: implementation of get nearest lanelet to Obstacle x, y, z coordinates
    // Reference: [FINDING]
    lanelet::ConstLanelet get_nearest_lanelet_to_xyz(float x, float y, float z);

    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point);

    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet);


    //void obstacle_subscriber();

    // TODO: function to retrieve the data from the obstacle message + accordingly create stop sign/ped/traffic light reg elem | DONE
    // Obstacle Message : https://github.com/WATonomous/wato_monorepo/blob/32946e5cbbc1721d404aa4851d58c7425b8121bc/src/wato_msgs/common_msgs/msg/Obstacle.msg
    void process_obstacle_msg(const common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr);

    // TODO: functions to add the three regulatory elements on the DRG
    // Old implementation: https://github.com/WATonomous/wato_monorepo_autodrive/blob/develop/src/path_planning/env_model/src/
    void add_stop_sign_reg_elem(lanelet::ConstLanelet reg_elem_lanelet);

    void add_pedestrian_reg_elem(lanelet::ConstLanelet reg_elem_lanelet);

    void add_traffic_light_reg_elem(lanelet::ConstLanelet reg_elem_lanelet);

  private:
    //rclcpp::Node::SharedPtr node_;
    //rclcpp::Subscription<common_msgs::msg::Obstacle>::SharedPtr subscription_;
    lanelet::LaneletMapPtr lanelet_ptr_;
    lanelet::routing::RoutingGraphPtr routing_graph_;
    std::shared_ptr<lanelet::Projector> projector_;
};

#endif
