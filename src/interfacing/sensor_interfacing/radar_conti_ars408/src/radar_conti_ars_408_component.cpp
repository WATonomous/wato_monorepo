#include "../include/radar_conti_ars_408_component.hpp"

#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#define _USE_MATH_DEFINES

#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "std_msgs/msg/string.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace watonomous {

const std::vector<FilterType> filterTypes = {
    FilterType::NOFOBJ,     FilterType::DISTANCE,    FilterType::AZIMUTH,  FilterType::VRELONCOME,
    FilterType::VRELDEPART, FilterType::RCS,         FilterType::LIFETIME, FilterType::SIZE,
    FilterType::PROBEXISTS, FilterType::Y,           FilterType::X,        FilterType::VYRIGHTLEFT,
    FilterType::VXONCOME,   FilterType::VYLEFTRIGHT, FilterType::VXDEPART,
    FilterType::UNKNOWN  // Add this to handle default case
};

radar_conti_ars408::radar_conti_ars408(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("radar_conti_ars408", options) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
radar_conti_ars408::on_configure(const rclcpp_lifecycle::State &) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
  auto node = shared_from_this();
  node->declare_parameter("radar_detection_topic_name",
                          rclcpp::ParameterValue("ars408/radar_detection"));
  node->declare_parameter("radar_packet_topic_name", rclcpp::ParameterValue("ars408/radar_packet"));
  node->declare_parameter("filter_config_topic_name",
                          rclcpp::ParameterValue("ars408/filter_config"));

  node->get_parameter("radar_detection_topic_name", radar_detection_topic_name_);
  node->get_parameter("radar_packet_topic_name", radar_packet_topic_name_);
  node->get_parameter("filter_config_topic_name", filter_config_topic_name_);

  auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(5))
                                 .reliability(rclcpp::ReliabilityPolicy::Reliable)
                                 .durability(rclcpp::DurabilityPolicy::TransientLocal);

  size_t topic_ind = 0;
  bool more_params = false;
  number_of_radars_ = 0;
  do {
    std::stringstream ss;
    ss << "radar_" << topic_ind;
    std::string radar_name = ss.str();

    node->declare_parameter(radar_name + ".link_name", rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (node->get_parameter(radar_name + ".link_name", parameter)) {
      more_params = true;
      filter_config_publishers_.push_back(
          this->create_publisher<radar_conti_ars408_msgs::msg::FilterStateCfg>(
              parameter.as_string() + "/" + filter_config_topic_name_, transient_local_qos));

      object_map_list_.push_back(std::map<int, radar_msgs::msg::RadarDetection>());
      object_list_list_.push_back(radar_msgs::msg::RadarPacket());
      radar_filter_configs_.push_back(radar_conti_ars408_msgs::msg::FilterStateCfg());
      radar_filter_active_.push_back(std::vector<bool>());
      radar_filter_valid_.push_back(std::vector<bool>());

      std::vector<bool> init_radar_active_values = {false, false, false, false, false,
                                                    false, false, false, false, false,
                                                    false, false, false, false, false};
      node->declare_parameter(radar_name + ".active",
                              rclcpp::ParameterValue(init_radar_active_values));
      node->get_parameter(radar_name + ".active", radar_filter_active_[topic_ind]);
      std::vector<bool> init_radar_valid_values = {false, false, false, false, false,
                                                   false, false, false, false, false,
                                                   false, false, false, false, false};
      node->declare_parameter(radar_name + ".valid",
                              rclcpp::ParameterValue(init_radar_valid_values));
      node->get_parameter(radar_name + ".valid", radar_filter_valid_[topic_ind]);

      radar_link_names_.push_back(parameter.as_string());

      RCLCPP_DEBUG(node->get_logger(), "radar frame is: %s", parameter.as_string().c_str());

      // Initialize Number of Objects Filters
      initializeFilterConfig<uint32_t>(radar_name, std::string("filtercfg_min_nofobj"), 0,
                                       radar_filter_configs_[topic_ind].filtercfg_min_nofobj.data);
      initializeFilterConfig<uint32_t>(radar_name, std::string("filtercfg_max_nofobj"), 20,
                                       radar_filter_configs_[topic_ind].filtercfg_max_nofobj.data);

      // Initialize Distance Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_distance"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_distance.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_distance"), 409.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_distance.data);

      // Initialize Azimuth Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_azimuth"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_azimuth.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_azimuth"), 100.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_azimuth.data);

      // Initialize Oncoming Velocity Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vreloncome"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_vreloncome.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vreloncome"), 128.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_vreloncome.data);

      // Initialize Departing Velocity Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vreldepart"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_vreldepart.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vreldepart"), 128.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_vreldepart.data);

      // Initialize Radar Cross Section Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_rcs"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_rcs.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_rcs"), 100.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_rcs.data);

      // Initialize Lifetime Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_lifetime"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_lifetime.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_lifetime"), 409.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_lifetime.data);

      // Initialize Size Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_size"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_size.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_size"), 102.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_size.data);

      // Initialize Probability of Existence Filters
      initializeFilterConfig<uint8_t>(
          radar_name, std::string("filtercfg_min_probexists"), 0,
          radar_filter_configs_[topic_ind].filtercfg_min_probexists.data);
      initializeFilterConfig<uint8_t>(
          radar_name, std::string("filtercfg_max_probexists"), 7,
          radar_filter_configs_[topic_ind].filtercfg_max_probexists.data);

      // Initialize Y Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_y"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_y.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_y"), 818.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_y.data);

      // Initialize X Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_x"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_x.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_x"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_x.data);

      // Initialize Y Velocity going left to right Filters
      initializeFilterConfig<float>(
          radar_name, std::string("filtercfg_min_vyrightleft"), 0.0,
          radar_filter_configs_[topic_ind].filtercfg_min_vyrightleft.data);
      initializeFilterConfig<float>(
          radar_name, std::string("filtercfg_max_vyrightleft"), 128.0,
          radar_filter_configs_[topic_ind].filtercfg_max_vyrightleft.data);

      // Initialize X Velocity oncoming Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vxoncome"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_vxoncome.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vxoncome"), 128.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_vxoncome.data);

      // Initialize Y Velocity going right to left Filters
      initializeFilterConfig<float>(
          radar_name, std::string("filtercfg_min_vyleftright"), 0.0,
          radar_filter_configs_[topic_ind].filtercfg_min_vyleftright.data);
      initializeFilterConfig<float>(
          radar_name, std::string("filtercfg_max_vyleftright"), 128.0,
          radar_filter_configs_[topic_ind].filtercfg_max_vyleftright.data);

      // Initialize X Velocity Departing Filters
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vxdepart"), 0.0,
                                    radar_filter_configs_[topic_ind].filtercfg_min_vxdepart.data);
      initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vxdepart"), 128.0,
                                    radar_filter_configs_[topic_ind].filtercfg_max_vxdepart.data);

      filter_config_initialized_list_.push_back(false);
      RCLCPP_WARN(this->get_logger(), "link_name is: %s", parameter.as_string().c_str());
      number_of_radars_++;
      topic_ind++;
    } else {
      more_params = false;
    }
  } while (more_params);

  can_frame_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", transient_local_qos,
      std::bind(&radar_conti_ars408::can_receive_callback, this, std::placeholders::_1));
  can_frame_publisher_ =
      this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", transient_local_qos);
  radar_packet_publisher_ =
      this->create_publisher<radar_msgs::msg::RadarPacket>("/radar_packet", transient_local_qos);

  object_count = 0.0;
  set_filter_service_ = create_service<radar_conti_ars408_msgs::srv::SetFilter>(
      "/set_filter", std::bind(&radar_conti_ars408::setFilterService, this, std::placeholders::_1,
                               std::placeholders::_2));

  generateUUIDTable();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template <typename T>
void radar_conti_ars408::initializeFilterConfig(std::string radar_name, std::string config_name,
                                                T value, T &config) {
  auto node = shared_from_this();
  T config_value;
  declare_parameter_with_type(node, radar_name + "." + config_name, value);
  get_parameter_with_type(node, radar_name + "." + config_name, config_value);
  config = config_value;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
radar_conti_ars408::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
radar_conti_ars408::on_error(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on error is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
radar_conti_ars408::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
radar_conti_ars408::on_deactivate(const rclcpp_lifecycle::State &) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
radar_conti_ars408::on_cleanup(const rclcpp_lifecycle::State &) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
  if (bond_) {
    bond_.reset();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

unique_identifier_msgs::msg::UUID radar_conti_ars408::generateRandomUUID() {
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
  return uuid;
}

void radar_conti_ars408::generateUUIDTable() {
  for (int i = 0; i <= (max_radar_id * number_of_radars_); i++) {
    UUID_table_.emplace_back(radar_conti_ars408::generateRandomUUID());
  }
}

void radar_conti_ars408::initializeFilterConfigs() {
  const int type = FilterCfg_FilterCfg_Type_Object;
  int min_value = 0;
  int max_value = 0;

  for (size_t radar_index = 0; radar_index < radar_filter_configs_.size(); radar_index++) {
    for (size_t filter_index = 0; filter_index < MAX_FilterState_Cfg_FilterState_Index;
         filter_index++) {
      int active = radar_filter_active_[radar_index][filter_index]
                       ? FilterCfg_FilterCfg_Active_active
                       : FilterCfg_FilterCfg_Active_inactive;
      int valid = radar_filter_valid_[radar_index][filter_index]
                      ? FilterCfg_FilterCfg_Valid_valid
                      : FilterCfg_FilterCfg_Valid_invalid;
      switch (filterTypes[filter_index]) {
        case FilterType::NOFOBJ:
          min_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_nofobj.data);
          max_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_nofobj.data);
          break;
        case FilterType::DISTANCE:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_distance.data / 0.1);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_distance.data / 0.1);
          break;
        case FilterType::AZIMUTH:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_azimuth.data / 0.025);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_azimuth.data / 0.025);
          break;
        case FilterType::VRELONCOME:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_vreloncome.data / 0.0315);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_vreloncome.data / 0.0315);
          break;
        case FilterType::VRELDEPART:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_vreldepart.data / 0.0315);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_vreldepart.data / 0.0315);
          break;
        case FilterType::RCS:
          min_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_rcs.data / 0.025);
          max_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_rcs.data / 0.025);
          break;
        case FilterType::LIFETIME:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_lifetime.data / 0.1);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_lifetime.data / 0.1);
          break;
        case FilterType::SIZE:
          min_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_size.data / 0.025);
          max_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_size.data / 0.025);
          break;
        case FilterType::PROBEXISTS:
          min_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_probexists.data);
          max_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_probexists.data);
          break;
        case FilterType::Y:
          min_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_y.data / 0.2);
          max_value =
              static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_y.data / 0.2);
          break;
        case FilterType::X:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_x.data);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_x.data);
          break;
        case FilterType::VYRIGHTLEFT:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_vyrightleft.data / 0.0315);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_vyrightleft.data / 0.0315);
          break;
        case FilterType::VXONCOME:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_vxoncome.data / 0.0315);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_vxoncome.data / 0.0315);
          break;
        case FilterType::VYLEFTRIGHT:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_vyleftright.data / 0.0315);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_vyleftright.data / 0.0315);
          break;
        case FilterType::VXDEPART:
          min_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_min_vxdepart.data / 0.0315);
          max_value = static_cast<int>(
              radar_filter_configs_[radar_index].filtercfg_max_vxdepart.data / 0.0315);
          break;
        default:
          break;
      }
      setFilter(radar_index, active, valid, type, filter_index, min_value, max_value);
      // need to sleep in order to read properly and not spam the can bus
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // initialized and publish once
    filter_config_initialized_list_[radar_index] = true;
    filter_config_publishers_[radar_index]->publish(radar_filter_configs_[radar_index]);
  }
}

void radar_conti_ars408::can_receive_callback(const can_msgs::msg::Frame::SharedPtr frame) {
  int sensor_id = Get_SensorID_From_MsgID(frame->id);

  // If the sensor_id is greater than the size of the number of object lists, break
  if (sensor_id > object_list_list_.size() - 1) {
    return;
  }

  // When a filter configuration message is sent, the sensor replies with the messages
  // FilterState_Header (0x203) with the number of configured filters and one message
  // FilterState_Cfg (0x204) for the filter that has been changed.
  if (Get_MsgID0_From_MsgID(frame->id) == ID_FilterState_Cfg) {
    updateFilterConfig(frame, sensor_id);
  }

  if (Get_MsgID0_From_MsgID(frame->id) == ID_RadarState) {
    operation_mode_ = CALC_RadarState_RadarState_OutputTypeCfg(
        GET_RadarState_RadarState_OutputTypeCfg(frame->data), 1.0);
  }

  // no output
  if (operation_mode_ == RadarState_RadarState_OutputTypeCfg_None) {
    return;
  }

  // object list
  if (operation_mode_ == RadarState_RadarState_OutputTypeCfg_SendObjects) {
    handle_object_list(frame);
  }
}

void radar_conti_ars408::handle_object_list(const can_msgs::msg::Frame::SharedPtr frame) {
  int sensor_id = Get_SensorID_From_MsgID(frame->id);

  if (Get_MsgID0_From_MsgID(frame->id) == ID_Obj_0_Status) {
    publish_object_map(sensor_id);
    object_list_list_[sensor_id].header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    object_map_list_[sensor_id].clear();
  }

  // Object General Information
  // for each Obj_1_General message a new object has to be created in the map
  if (Get_MsgID0_From_MsgID(frame->id) == ID_Obj_1_General) {
    radar_msgs::msg::RadarDetection o;

    // object ID
    int id = GET_Obj_1_General_Obj_ID(frame->data);
    o.obj_id = GET_Obj_1_General_Obj_ID(frame->data);
    o.sensor_id = Get_SensorID_From_MsgID(frame->id);

    // longitudinal distance
    o.pos_x = CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(frame->data), 1.0);

    // lateral distance
    o.pos_y = CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(frame->data), 1.0);

    // relative longitudinal velocity
    double long_vel =
        CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(frame->data), 1.0);

    // relative lateral velocity
    double lat_vel =
        CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(frame->data), 1.0);

    // radial velocity
    double angle_det = atan2(o.pos_y, o.pos_x);
    double radial_vel = long_vel * cos(angle_det) + lat_vel * sin(angle_det);
    o.vrel_rad = radial_vel;

    o.rcs0 = CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(frame->data), 1.0);

    // insert object into map
    object_map_list_[sensor_id].insert(std::pair<int, radar_msgs::msg::RadarDetection>(id, o));
  }

  // Object Quality Information
  // for each Obj_2_Quality message the existing object in the map has to be updated
  if (Get_MsgID0_From_MsgID(frame->id) == ID_Obj_2_Quality) {
  }

  // Object Extended Information
  // for each Obj_3_ExtInfo message the existing object in the map has to be updated
  if (Get_MsgID0_From_MsgID(frame->id) == ID_Obj_3_Extended) {
  };
}

void radar_conti_ars408::publish_object_map(int sensor_id) {
  radar_msgs::msg::RadarPacket packet;
  packet.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
  packet.header.frame_id = radar_link_names_[sensor_id];

  std::map<int, radar_msgs::msg::RadarDetection>::iterator itr;

  for (itr = object_map_list_[sensor_id].begin(); itr != object_map_list_[sensor_id].end(); ++itr) {
    packet.detections.emplace_back(itr->second);
  }
  radar_packet_publisher_->publish(packet);
}

void radar_conti_ars408::setFilterService(
    const std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Request> request,
    std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Response> response) {
  auto req = *request;
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!setFilter(req.sensor_id, FilterCfg_FilterCfg_Active_active, FilterCfg_FilterCfg_Valid_valid,
                 req.type, req.index, req.min_value, req.max_value)) {
    response->success = false;
    return;
  }
  response->success = true;
}

bool radar_conti_ars408::setFilter(const int &sensor_id, const int &active, const int &valid,
                                   const int &type, const int &index, const int &min_value,
                                   const int &max_value) {
  uint32_t msg_id = ID_FilterCfg;
  Set_SensorID_In_MsgID(msg_id, sensor_id);

  std::array<unsigned char, CAN_MAX_DLC> data;
  SET_FilterCfg_FilterCfg_Active(data, active);
  SET_FilterCfg_FilterCfg_Valid(data, valid);
  SET_FilterCfg_FilterCfg_Type(data, type);
  SET_FilterCfg_FilterCfg_Index(data, index);

  switch (filterTypes[index]) {
    case FilterType::NOFOBJ:
      RCLCPP_DEBUG(this->get_logger(), "Setting Number Of Objects Filter");
      SET_FilterCfg_FilterCfg_Max_NofObj(data, max_value);
      SET_FilterCfg_FilterCfg_Min_NofObj(data, min_value);
      break;
    case FilterType::DISTANCE:
      RCLCPP_DEBUG(this->get_logger(), "Setting Distance Filter");
      SET_FilterCfg_FilterCfg_Max_Distance(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Distance(data, min_value);
      break;
    case FilterType::AZIMUTH:
      RCLCPP_DEBUG(this->get_logger(), "Setting Azimuth Filter");
      SET_FilterCfg_FilterCfg_Max_Azimuth(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Azimuth(data, min_value);
      break;
    case FilterType::VRELONCOME:
      RCLCPP_DEBUG(this->get_logger(), "Setting Oncoming Velocity Filter");
      SET_FilterCfg_FilterCfg_Max_VrelOncome(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VrelOncome(data, min_value);
      break;
    case FilterType::VRELDEPART:
      RCLCPP_DEBUG(this->get_logger(), "Setting Departing Velocity Filter");
      SET_FilterCfg_FilterCfg_Max_VrelDepart(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VrelDepart(data, min_value);
      break;
    case FilterType::RCS:
      RCLCPP_DEBUG(this->get_logger(), "Setting RCS Filter");
      SET_FilterCfg_FilterCfg_Max_RCS(data, max_value);
      SET_FilterCfg_FilterCfg_Min_RCS(data, min_value);
      break;
    case FilterType::LIFETIME:
      RCLCPP_DEBUG(this->get_logger(), "Setting Lifetime Filter");
      SET_FilterCfg_FilterCfg_Max_Lifetime(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Lifetime(data, min_value);
      break;
    case FilterType::SIZE:
      RCLCPP_DEBUG(this->get_logger(), "Setting Size Filter");
      SET_FilterCfg_FilterCfg_Max_Size(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Size(data, min_value);
      break;
    case FilterType::PROBEXISTS:
      RCLCPP_DEBUG(this->get_logger(), "Setting Probability of Existence Filter");
      SET_FilterCfg_FilterCfg_Max_ProbExists(data, max_value);
      SET_FilterCfg_FilterCfg_Min_ProbExists(data, min_value);
      break;
    case FilterType::Y:
      RCLCPP_DEBUG(this->get_logger(), "Setting Y Filter");
      SET_FilterCfg_FilterCfg_Max_Y(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Y(data, min_value);
      break;
    case FilterType::X:
      // TODO: MAKE THIS 13BIT
      RCLCPP_DEBUG(this->get_logger(), "X Filter currently not implemented");
      return false;
    case FilterType::VYRIGHTLEFT:
      RCLCPP_DEBUG(this->get_logger(), "Setting Right Left Filter");
      SET_FilterCfg_FilterCfg_Max_VYRightLeft(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VYRightLeft(data, min_value);
      break;
    case FilterType::VXONCOME:
      RCLCPP_DEBUG(this->get_logger(), "Setting X Oncoming Filter");
      SET_FilterCfg_FilterCfg_Max_VXOncome(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VXOncome(data, min_value);
      break;
    case FilterType::VYLEFTRIGHT:
      RCLCPP_DEBUG(this->get_logger(), "Setting Left Right Filter");
      SET_FilterCfg_FilterCfg_Max_VYLeftRight(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VYLeftRight(data, min_value);
      break;
    case FilterType::VXDEPART:
      RCLCPP_DEBUG(this->get_logger(), "Setting X Departing Filter");
      SET_FilterCfg_FilterCfg_Max_VXDepart(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VXDepart(data, min_value);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown Filter Index");
      return false;
  }

  RCLCPP_DEBUG(this->get_logger(), "valid: %i", valid);
  RCLCPP_DEBUG(this->get_logger(), "active: %i", active);
  RCLCPP_DEBUG(this->get_logger(), "min_value is: %i", min_value);
  RCLCPP_DEBUG(this->get_logger(), "max_value is: %i", max_value);

  can_msgs::msg::Frame frame;
  frame.data = data;
  can_frame_publisher_->publish(frame);

  return true;
}

void radar_conti_ars408::updateFilterConfig(const can_msgs::msg::Frame::SharedPtr frame,
                                            const int &sensor_id) {
  radar_filter_configs_[sensor_id].header.stamp = rclcpp_lifecycle::LifecycleNode::now();
  radar_filter_configs_[sensor_id].header.frame_id = radar_link_names_[sensor_id];

  radar_filter_configs_[sensor_id].filtercfg_type.data =
      CALC_FilterState_Cfg_FilterState_Type(GET_FilterState_Cfg_FilterState_Type(frame->data), 1.0);
  int index = CALC_FilterState_Cfg_FilterState_Index(
      GET_FilterState_Cfg_FilterState_Index(frame->data), 1.0);

  switch (filterTypes[index]) {
    case FilterType::NOFOBJ:
      radar_filter_configs_[sensor_id].filtercfg_min_nofobj.data =
          CALC_FilterState_Cfg_FilterState_Min_NofObj(
              GET_FilterState_Cfg_FilterState_Min_NofObj(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_nofobj.data =
          CALC_FilterState_Cfg_FilterState_Max_NofObj(
              GET_FilterState_Cfg_FilterState_Max_NofObj(frame->data), 1.0);
      break;
    case FilterType::DISTANCE:
      radar_filter_configs_[sensor_id].filtercfg_min_distance.data =
          CALC_FilterState_Cfg_FilterState_Min_Distance(
              GET_FilterState_Cfg_FilterState_Min_Distance(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_distance.data =
          CALC_FilterState_Cfg_FilterState_Max_Distance(
              GET_FilterState_Cfg_FilterState_Max_Distance(frame->data), 1.0);
      break;
    case FilterType::AZIMUTH:
      radar_filter_configs_[sensor_id].filtercfg_min_azimuth.data =
          CALC_FilterState_Cfg_FilterState_Min_Azimuth(
              GET_FilterState_Cfg_FilterState_Min_Azimuth(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_azimuth.data =
          CALC_FilterState_Cfg_FilterState_Max_Azimuth(
              GET_FilterState_Cfg_FilterState_Max_Azimuth(frame->data), 1.0);
      break;
    case FilterType::VRELONCOME:
      radar_filter_configs_[sensor_id].filtercfg_min_vreloncome.data =
          CALC_FilterState_Cfg_FilterState_Min_VrelOncome(
              GET_FilterState_Cfg_FilterState_Min_VrelOncome(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vreloncome.data =
          CALC_FilterState_Cfg_FilterState_Max_VrelOncome(
              GET_FilterState_Cfg_FilterState_Max_VrelOncome(frame->data), 1.0);
      break;
    case FilterType::VRELDEPART:
      radar_filter_configs_[sensor_id].filtercfg_min_vreldepart.data =
          CALC_FilterState_Cfg_FilterState_Min_VrelDepart(
              GET_FilterState_Cfg_FilterState_Min_VrelDepart(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vreldepart.data =
          CALC_FilterState_Cfg_FilterState_Max_VrelDepart(
              GET_FilterState_Cfg_FilterState_Max_VrelDepart(frame->data), 1.0);
      break;
    case FilterType::RCS:
      radar_filter_configs_[sensor_id].filtercfg_min_rcs.data =
          CALC_FilterState_Cfg_FilterState_Min_RCS(
              GET_FilterState_Cfg_FilterState_Min_RCS(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_rcs.data =
          CALC_FilterState_Cfg_FilterState_Max_RCS(
              GET_FilterState_Cfg_FilterState_Max_RCS(frame->data), 1.0);
      break;
    case FilterType::LIFETIME:
      radar_filter_configs_[sensor_id].filtercfg_min_lifetime.data =
          CALC_FilterState_Cfg_FilterState_Min_Lifetime(
              GET_FilterState_Cfg_FilterState_Min_Lifetime(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_lifetime.data =
          CALC_FilterState_Cfg_FilterState_Max_Lifetime(
              GET_FilterState_Cfg_FilterState_Max_Lifetime(frame->data), 1.0);
      break;
    case FilterType::SIZE:
      radar_filter_configs_[sensor_id].filtercfg_min_size.data =
          CALC_FilterState_Cfg_FilterState_Min_Size(
              GET_FilterState_Cfg_FilterState_Min_Size(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_size.data =
          CALC_FilterState_Cfg_FilterState_Max_Size(
              GET_FilterState_Cfg_FilterState_Max_Size(frame->data), 1.0);
      break;
    case FilterType::PROBEXISTS:
      radar_filter_configs_[sensor_id].filtercfg_min_probexists.data =
          CALC_FilterState_Cfg_FilterState_Min_ProbExists(
              GET_FilterState_Cfg_FilterState_Min_ProbExists(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_probexists.data =
          CALC_FilterState_Cfg_FilterState_Max_ProbExists(
              GET_FilterState_Cfg_FilterState_Max_ProbExists(frame->data), 1.0);
      break;
    case FilterType::Y:
      radar_filter_configs_[sensor_id].filtercfg_min_y.data =
          CALC_FilterState_Cfg_FilterState_Min_Y(GET_FilterState_Cfg_FilterState_Min_Y(frame->data),
                                                 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_y.data =
          CALC_FilterState_Cfg_FilterState_Max_Y(GET_FilterState_Cfg_FilterState_Max_Y(frame->data),
                                                 1.0);
      break;
    case FilterType::X:
      radar_filter_configs_[sensor_id].filtercfg_min_x.data =
          CALC_FilterState_Cfg_FilterState_Min_X(GET_FilterState_Cfg_FilterState_Min_X(frame->data),
                                                 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_x.data =
          CALC_FilterState_Cfg_FilterState_Max_X(GET_FilterState_Cfg_FilterState_Max_X(frame->data),
                                                 1.0);
      break;
    case FilterType::VYRIGHTLEFT:
      radar_filter_configs_[sensor_id].filtercfg_min_vyrightleft.data =
          CALC_FilterState_Cfg_FilterState_Min_VYRightLeft(
              GET_FilterState_Cfg_FilterState_Min_VYRightLeft(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vyrightleft.data =
          CALC_FilterState_Cfg_FilterState_Max_VYRightLeft(
              GET_FilterState_Cfg_FilterState_Max_VYRightLeft(frame->data), 1.0);
      break;
    case FilterType::VXONCOME:
      radar_filter_configs_[sensor_id].filtercfg_min_vxoncome.data =
          CALC_FilterState_Cfg_FilterState_Min_VXOncome(
              GET_FilterState_Cfg_FilterState_Min_VXOncome(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vxoncome.data =
          CALC_FilterState_Cfg_FilterState_Max_VXOncome(
              GET_FilterState_Cfg_FilterState_Max_VXOncome(frame->data), 1.0);
      break;
    case FilterType::VYLEFTRIGHT:
      radar_filter_configs_[sensor_id].filtercfg_min_vyleftright.data =
          CALC_FilterState_Cfg_FilterState_Min_VYLeftRight(
              GET_FilterState_Cfg_FilterState_Min_VYLeftRight(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vyleftright.data =
          CALC_FilterState_Cfg_FilterState_Max_VYLeftRight(
              GET_FilterState_Cfg_FilterState_Max_VYLeftRight(frame->data), 1.0);
      break;
    case FilterType::VXDEPART:
      radar_filter_configs_[sensor_id].filtercfg_min_vxdepart.data =
          CALC_FilterState_Cfg_FilterState_Min_VXDepart(
              GET_FilterState_Cfg_FilterState_Min_VXDepart(frame->data), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vxdepart.data =
          CALC_FilterState_Cfg_FilterState_Max_VXDepart(
              GET_FilterState_Cfg_FilterState_Max_VXDepart(frame->data), 1.0);
      break;
    default:
      break;
  }
  // only publish once we have initialized values in the filter config by sending invalid values for
  // a response
  if (filter_config_initialized_list_[sensor_id]) {
    filter_config_publishers_[sensor_id]->publish(radar_filter_configs_[sensor_id]);
  }
}

}  // namespace watonomous

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(watonomous::radar_conti_ars408)
