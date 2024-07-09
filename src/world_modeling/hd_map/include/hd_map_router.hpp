#ifndef WORLD_MODELING_HD_MAP_ROUTER_
#define WORLD_MODELING_HD_MAP_ROUTER_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "common_msgs/msg/obstacle.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "vision_msgs/msg/detection_3d.hpp"
#include "vision_msgs/msg/detection_3d_array.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BoundingBox3d.h>
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
#include "traffic_light_reg_elem.hpp"
#include <unordered_set>
#include <map>

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
    
    // Reference: NovAtel Co-ordinate System (https://docs.novatel.com/OEM7/Content/SPAN_Operation/Definition_Reference_Frames.htm)
    lanelet::ConstLanelet get_nearest_lanelet_to_xy(float x, float y, float width_x, float height_y);
    lanelet::ConstLanelet get_nearest_lanelet_to_xyz(float x, float y, float z);

    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point);

    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet);


    std::string get_detection3d_class(const vision_msgs::msg::Detection3D::SharedPtr reg_elem_msg_ptr);
traffic_light_id
    // Obstacle Message : https://github.com/WATonomous/wato_monorepo/blob/32946e5cbbc1721d404aa4851d58c7425b8121bc/src/wato_msgs/common_msgs/msg/Obstacle.msg
    void process_traffic_light_msg(const vision_msgs::msg::Detection3DArray::SharedPtr traffic_light_array_msg_ptr);
    void process_traffic_sign_msg(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
    void process_pedestrian_msg(const vision_msgs::msg::Detection3DArray::SharedPtr obstacle_msg_ptr);

    // TODO: functions to add the regulatory elements on the DRG
    // Old implementation: https://github.com/WATonomous/wato_monorepo_autodrive/blob/develop/src/path_planning/env_model/src/
    void add_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
    void add_stop_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
    void add_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr);
    void add_pedestrian(const vision_msgs::msg::Detection3D::SharedPtr obstacle_msg_ptr);

    // TODO: functions to update the regulatory elements on the DRG
    void update_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
    void update_stop_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
    void update_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr);
    void update_pedestrian(const vision_msgs::msg::Detection3D::SharedPtr obstacle_msg_ptr);

    // TODO: functions to remove the regulatory elements on the DRG
    void remove_traffic_light(uint64_t traffic_light_id);
    void remove_pedestrian(uint64_t pedestrian_id);

  private:
    lanelet::LaneletMapPtr lanelet_ptr_;
    lanelet::routing::RoutingGraphPtr routing_graph_;
    std::shared_ptr<lanelet::Projector> projector_;

    std::unordered_set<uint64_t> traffic_sign_list_;
    std::unordered_set<uint64_t> traffic_light_list_;
    std::unordered_set<uint64_t> pedestrian_list_;
};

#endif