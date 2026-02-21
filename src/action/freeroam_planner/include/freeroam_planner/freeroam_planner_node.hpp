#ifndef FREEROAM_PLANNER__FREEROAM_PLANNER_NODE_HPP_
#define FREEROAM_PLANNER__FREEROAM_PLANNER_NODE_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace freeroam_planner
{

class FreeroamPlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit FreeroamPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void planCallback();
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // Parameters
  std::string goal_topic_;
  std::string costmap_topic_;
  std::string trajectory_topic_;
  std::string base_frame_;
  double max_speed_;
  double goal_tolerance_;
  int obstacle_threshold_;
  bool allow_diagonal_;
  double planning_rate_hz_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Pub/Sub
  rclcpp_lifecycle::LifecyclePublisher<wato_trajectory_msgs::msg::Trajectory>::SharedPtr
    trajectory_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;

  // State
  geometry_msgs::msg::PointStamped::SharedPtr latest_goal_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
};

}  // namespace freeroam_planner

#endif  // FREEROAM_PLANNER__FREEROAM_PLANNER_NODE_HPP_
