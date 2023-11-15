#ifndef WORLD_MODELING_HD_MAP_ROUTER_
#define WORLD_MODELING_HD_MAP_ROUTER_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "common_msgs/msg/Obstacle.hpp"
#include "geometry_msgs/msg/PoseWithCovariance.hpp"
#include "geometry_msgs/msg/Pose.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Exceptions.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

lanelet::GPSPoint ros_gps_msg_to_lanelet_gps_point(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

class HDMapRouter {
  public:
    HDMapRouter(rclcpp::Node::SharedPtr node);

    bool set_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr);

    bool set_projector(std::shared_ptr<lanelet::Projector> projector);

    lanelet::LaneletMapPtr get_lanelet();

    lanelet::BasicPoint3d project_gps_to_point3d(lanelet::GPSPoint gps_point);
    lanelet::GPSPoint project_point3d_to_gps(lanelet::BasicPoint3d point3d);

    lanelet::ConstLanelet get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point);

    lanelet::ConstLanelet get_nearest_lanelet_to_xyz(float x, float y, float z);

    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point);
    
    // TODO: implementation of route function to find shortest path from from_lanelet to to_lanelet
    // Reference: https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/06_routing/main.cpp
    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet);


    void obstacle_subscription();

    // TODO: function to retrieve the data from the obstacle message + accordingly create stop sign/ped/traffic light reg elem | DONE
    // Obstacle Message : https://github.com/WATonomous/wato_monorepo/blob/32946e5cbbc1721d404aa4851d58c7425b8121bc/src/wato_msgs/common_msgs/msg/Obstacle.msg
    void process_obstacle_msg(const common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr);

    // TODO: functions to add the three regulatory elements on the DRG
    // Old implementation: https://github.com/WATonomous/wato_monorepo_autodrive/blob/develop/src/path_planning/env_model/src/
    void add_stop_sign_reg_elem(lanelet::ConstLanelet reg_elem_lanelet);

    void add_ped_reg_elem(lanelet::ConstLanelet reg_elem_lanelet);

    void add_traffic_light_reg_elem(lanelet::ConstLanelet reg_elem_lanelet);

  private:
    lanelet::LaneletMapPtr lanelet_ptr_;
    lanelet::routing::RoutingGraphPtr routing_graph_;
    std::shared_ptr<lanelet::Projector> projector_;
};

#endif
