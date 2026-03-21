#include "eidos/core/plugin_registry.hpp"
#include "eidos/core/map_manager.hpp"

namespace eidos {

void PluginRegistry::loadAll(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer* tf,
    MapManager* map_manager,
    const LockFreePose* estimator_pose,
    const std::atomic<SlamState>* state,
    const AtomicSlot<gtsam::Values>* estimator_values) {
  loadFactorPlugins(node, tf, map_manager, estimator_pose, state);
  loadMotionModel(node, tf, state);
  loadRelocalizationPlugins(node, tf, map_manager);
  loadVisualizationPlugins(node, tf, map_manager, estimator_values);
}

void PluginRegistry::activateAll() {
  for (auto& p : factor_plugins) p->activate();
  if (motion_model) motion_model->activate();
  for (auto& p : reloc_plugins) p->activate();
  for (auto& p : vis_plugins) p->activate();
}

void PluginRegistry::deactivateAll() {
  for (auto& p : factor_plugins) p->deactivate();
  if (motion_model) motion_model->deactivate();
  for (auto& p : reloc_plugins) p->deactivate();
  for (auto& p : vis_plugins) p->deactivate();
}

void PluginRegistry::loadFactorPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, tf2_ros::Buffer* tf,
    MapManager* map_manager,
    const LockFreePose* estimator_pose, const std::atomic<SlamState>* state) {
  factor_loader_ = std::make_unique<pluginlib::ClassLoader<FactorPlugin>>(
      "eidos_v2", "eidos::FactorPlugin");

  std::vector<std::string> plugin_names;
  node->get_parameter("factor_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    node->declare_parameter(name + ".plugin", std::string{});
    node->get_parameter(name + ".plugin", plugin_type);

    if (plugin_type.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Factor plugin '%s' has no plugin type", name.c_str());
      continue;
    }

    auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto plugin = factor_loader_->createSharedInstance(plugin_type);
    plugin->initialize(name, node, tf, cb_group, map_manager, estimator_pose, state);
    factor_plugins.push_back(plugin);

    RCLCPP_INFO(node->get_logger(), "Loaded factor plugin: %s (%s)", name.c_str(), plugin_type.c_str());
  }
}

void PluginRegistry::loadMotionModel(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, tf2_ros::Buffer* tf,
    const std::atomic<SlamState>* state) {
  motion_model_loader_ = std::make_unique<pluginlib::ClassLoader<MotionModelPlugin>>(
      "eidos_v2", "eidos::MotionModelPlugin");

  std::string plugin_type;
  node->get_parameter("motion_model.plugin", plugin_type);

  if (plugin_type.empty()) {
    RCLCPP_WARN(node->get_logger(), "No motion model configured");
    return;
  }

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  motion_model = motion_model_loader_->createSharedInstance(plugin_type);
  motion_model->initialize("motion_model", node, tf, cb_group, state);

  RCLCPP_INFO(node->get_logger(), "Loaded motion model: %s", plugin_type.c_str());
}

void PluginRegistry::loadRelocalizationPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, tf2_ros::Buffer* tf,
    MapManager* map_manager) {
  reloc_loader_ = std::make_unique<pluginlib::ClassLoader<RelocalizationPlugin>>(
      "eidos_v2", "eidos::RelocalizationPlugin");

  std::vector<std::string> plugin_names;
  node->get_parameter("relocalization_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    node->declare_parameter(name + ".plugin", std::string{});
    node->get_parameter(name + ".plugin", plugin_type);
    if (plugin_type.empty()) continue;

    auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto plugin = reloc_loader_->createSharedInstance(plugin_type);
    plugin->initialize(name, node, tf, cb_group, map_manager);
    reloc_plugins.push_back(plugin);

    RCLCPP_INFO(node->get_logger(), "Loaded relocalization plugin: %s (%s)",
                name.c_str(), plugin_type.c_str());
  }
}

void PluginRegistry::loadVisualizationPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, tf2_ros::Buffer* tf,
    MapManager* map_manager,
    const AtomicSlot<gtsam::Values>* estimator_values) {
  vis_loader_ = std::make_unique<pluginlib::ClassLoader<VisualizationPlugin>>(
      "eidos_v2", "eidos::VisualizationPlugin");

  std::vector<std::string> plugin_names;
  node->get_parameter("visualization_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    node->declare_parameter(name + ".plugin", std::string{});
    node->get_parameter(name + ".plugin", plugin_type);
    if (plugin_type.empty()) continue;

    auto plugin = vis_loader_->createSharedInstance(plugin_type);
    plugin->initialize(name, node, tf, map_manager, estimator_values, 1.0);
    vis_plugins.push_back(plugin);

    RCLCPP_INFO(node->get_logger(), "Loaded visualization plugin: %s (%s)",
                name.c_str(), plugin_type.c_str());
  }
}

}  // namespace eidos
