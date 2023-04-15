#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/io/XmlIO.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"

using DistLanelet = std::pair<double, lanelet::ConstLanelet>;
using DistLaneletList = std::vector<DistLanelet>;

class EnvModel : public rclcpp::Node {
public:
  EnvModel() : Node("env_model") {
    // Load lanelet map
    lanelet::io_handlers::XmlReader reader;
    lanelet::ErrorMessages errors;
    lanelet::LaneletMapPtr map = reader("lanelet_map.osm", errors);
    if (!errors.empty()) {
      throw std::runtime_error("Failed to load map");
    }

    lanelet_map_ = map;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                     std::bind(&EnvModel::getEgoLanelets, this));
  }

  DistLaneletList getEgoLanelets() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(kMapFrameName, kVehicleFrameName, tf2::TimePoint(), std::chrono::duration<double>(5.0));
    } catch (tf2::TransformException &ex) {
      return {};
    }
  
    auto ego_lanelets = lanelet::geometry::findWithin2d(
        lanelet_map_->laneletLayer,
        lanelet::Point2d{lanelet::InvalId, tf.transform.translation.x,
                         tf.transform.translation.y});

    DistLaneletList dist_lanelets;
    for (const auto &lanelet : ego_lanelets) {
      double distance = lanelet::geometry::distance(lanelet, tf.transform.translation.x,
                                                     tf.transform.translation.y);
      dist_lanelets.emplace_back(distance, lanelet);
    }

    // sort by distance
    std::sort(dist_lanelets.begin(), dist_lanelets.end(),
              [](const DistLanelet &lhs, const DistLanelet &rhs) {
                return lhs.first < rhs.first;
              });

    return dist_lanelets;
  }

private:
  lanelet::LaneletMapPtr lanelet_map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::string kMapFrameName = "map";
  const std::string kVehicleFrameName = "vehicle";
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvModel>());
  rclcpp::shutdown();
  return 0;
}
