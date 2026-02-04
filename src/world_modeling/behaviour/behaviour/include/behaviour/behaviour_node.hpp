#ifndef BEHAVIOUR__BEHAVIOUR_NODE_HPP_
#define BEHAVIOUR__BEHAVIOUR_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lanelet_msgs/msg/current_lane_context.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "behaviour/behaviour_tree.hpp"
#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/area_occupancy_store.hpp"

namespace behaviour
{
  /**
   * @class BehaviourNode
   * @brief A standard node that manages Eve's behavior tree and handles world state updates.
   */
  class BehaviourNode : public rclcpp::Node
  {
  public:
    /**
     * @brief Initializes parameters, TF listeners, subscribers, and the behavior tree.
     * @param options Node options for configuration.
     */
    explicit BehaviourNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Virtual destructor.
     */
    virtual ~BehaviourNode() = default;

    /**
     * @brief Creates and initializes the behavior tree after node construction.
     */
    void init();

  private:
    /**
     * @brief Time callback that ticks the behavior tree
     */
    void tickTreeTimerCallback();

    /**
     * @brief Timer callback to look up the latest TF transform and updates the blackboard.
     */
    void tfTimerCallback();

    // Core component
    std::shared_ptr<BehaviourTree> tree_;
    std::shared_ptr<DynamicObjectStore> dynamic_objects_store_;
    std::shared_ptr<AreaOccupancyStore> area_occupancy_store_;

    // TF for pose and velocity
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS Communications
    rclcpp::TimerBase::SharedPtr tick_tree_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Subs
    rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr current_lane_context_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_point_sub_;
    rclcpp::Subscription<world_model_msgs::msg::WorldObjectArray>::SharedPtr dynamic_objects_sub_;
    rclcpp::Subscription<world_model_msgs::msg::AreaOccupancy>::SharedPtr area_occupancy_sub_;

    // Configuration
    std::string map_frame_;
    std::string base_frame_;

    // Check if last transform was found
    bool has_last_tf_;

    // Vars to store previous transform
    rclcpp::Time last_time_;
    tf2::Vector3 last_position_;
    tf2::Quaternion last_orientation_;
  };
} // namespace behaviour

#endif // BEHAVIOUR__BEHAVIOUR_NODE_HPP_