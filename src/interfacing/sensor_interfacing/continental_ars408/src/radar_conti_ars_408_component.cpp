#include "../include/radar_conti_ars_408_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <math.h>

#define _USE_MATH_DEFINES

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace FHAC
{

radar_conti_ars408::radar_conti_ars408(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("radar_conti_ars408", options)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_configure(
    const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
  auto node = shared_from_this();
  node->declare_parameter("can_channel", rclcpp::ParameterValue(""));
  node->declare_parameter("radar_packet_topic_name", rclcpp::ParameterValue("ars408/RadarPacket"));

  node->get_parameter("can_channel", can_channel_);
  node->get_parameter("radar_packet_topic_name", radar_packet_topic_name_);

  if (can_channel_.empty())
  {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "No can_channel_ specified.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  size_t topic_ind = 0;
  bool more_params = false;
  number_of_radars_ = 0;
  do
  {
    // Build the string in the form of "radar_link_X", where X is the sensor ID of
    // the rader on the CANBUS, then check if we have any parameters with that value. Users need
    // to make sure they don't have gaps in their configs (e.g.,footprint0 and then
    // footprint2)
    std::stringstream ss;
    ss << "radar_link_" << topic_ind++;
    std::string radar_link_name = ss.str();
    node->declare_parameter(radar_link_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (node->get_parameter(radar_link_name, parameter))
    {
      more_params = true;
      radar_link_names_.push_back(parameter.as_string());
      radar_packet_publishers_.push_back(this->create_publisher<radar_msgs::msg::RadarPacket>(parameter.as_string() + "/" + radar_packet_topic_name_, qos));
      
      RCLCPP_WARN(this->get_logger(), "link_name is: %s", parameter.as_string().c_str());
      number_of_radars_++;
    }
    else
    {
      more_params = false;
    }
  } while (more_params);

  canChannel0.Init(can_channel_.c_str(), std::bind(&radar_conti_ars408::can_receive_callback, this, _1));
  object_count = 0.0;

  generateUUIDTable();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on error is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_activate(
    const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < radar_packet_publishers_.size(); i++)
  {
    radar_packet_publishers_[i]->on_activate();
  }

  bond_ = std::make_unique<bond::Bond>(std::string("bond"), this->get_name(), shared_from_this());
  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
  if (bond_)
  {
    bond_.reset();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

unique_identifier_msgs::msg::UUID radar_conti_ars408::generateRandomUUID()
{
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
  return uuid;
}

void radar_conti_ars408::generateUUIDTable()
{
  for (int i = 0; i <= (max_radar_id * number_of_radars_); i++)
  {
    UUID_table_.emplace_back(radar_conti_ars408::generateRandomUUID());
  }
}

void radar_conti_ars408::can_receive_callback(const can_msgs::msg::Frame msg)
{

  int sensor_id = Get_SensorID_From_MsgID(msg.id);

  // If the sensor_id is greater than the size of the number of object lists, break
  if (sensor_id > radar_packet_publishers_.size() - 1)
  {
    return;
  }

  if (Get_MsgID0_From_MsgID(msg.id) == ID_RadarState)
  {
    operation_mode_ = CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(msg.data), 1.0);
  }

  // no output
  if (operation_mode_ == RadarState_RadarState_OutputTypeCfg_None)
  {
    return;
  }

  // object list
  if (operation_mode_ == RadarState_RadarState_OutputTypeCfg_SendObjects)
  {
    handle_object_list(msg);
  }
}

// [TODO] ADD ALL FIELDS IN RADAR PACKET/ RADAR DETECTION
void radar_conti_ars408::handle_object_list(const can_msgs::msg::Frame msg)
{

  int sensor_id = Get_SensorID_From_MsgID(msg.id);

  if (Get_MsgID0_From_MsgID(msg.id) == ID_Obj_0_Status)
  {
    publish_object_map(sensor_id);
    // [ TODO ] NOT NEEDED??
    // object_list_list_[sensor_id].header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    // object_list_list_[sensor_id].object_count.data = GET_Obj_0_Status_Obj_NofObjects(msg.data);
    // object_map_list_[sensor_id].clear();
  }

  // Object General Information
  // for each Obj_1_General message a new object has to be created in the map
  if (Get_MsgID0_From_MsgID(msg.id) == ID_Obj_1_General)
  {
    radar_msgs::msg::RadarDetection r;

    // object ID
    int id = GET_Obj_1_General_Obj_ID(msg.data);
    r.obj_id = id;

    // longitudinal distance
    r.pos_x =
        CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(msg.data), 1.0);

    // lateral distance
    r.pos_y =
        CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(msg.data), 1.0);

    // relative longitudinal velocity
    double long_vel =
        CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(msg.data), 1.0);

    double lat_vel = CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(msg.data), 1.0);

    double angle_det = atan2(r.pos_y, r.pos_x);
    double radial_vel = long_vel * cos(angle_det) + lat_vel * sin(angle_det);

    r.vrel_rad = radial_vel;

    r.rcs0 =
        CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(msg.data), 1.0);

    // insert object into map (mapped by its obj id)
    object_map_list_[sensor_id].insert(std::make_pair(id, r));
  }

  // Object Quality Information
  // for each Obj_2_Quality message the existing object in the map has to be updated
  if (Get_MsgID0_From_MsgID(msg.id) == ID_Obj_2_Quality)
  {

    RCLCPP_INFO(this->get_logger(), "Received Object_2_Quality msg (0x60c)");

    /* int id = GET_Obj_2_Quality_Obj_ID(msg.data);

    object_map_list_[sensor_id][id].object_quality.obj_distlong_rms.data =
        CALC_Obj_2_Quality_Obj_DistLong_rms(GET_Obj_2_Quality_Obj_DistLong_rms(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_quality.obj_distlat_rms.data =
        CALC_Obj_2_Quality_Obj_DistLat_rms(GET_Obj_2_Quality_Obj_DistLat_rms(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_quality.obj_vrellong_rms.data =
        CALC_Obj_2_Quality_Obj_VrelLong_rms(GET_Obj_2_Quality_Obj_VrelLong_rms(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_quality.obj_vrellat_rms.data =
        CALC_Obj_2_Quality_Obj_VrelLat_rms(GET_Obj_2_Quality_Obj_VrelLat_rms(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_quality.obj_probofexist.data =
        CALC_Obj_2_Quality_Obj_ProbOfExist(GET_Obj_2_Quality_Obj_ProbOfExist(msg.data), 1.0); */
  }

  // Object Extended Information
  // for each Obj_3_ExtInfo message the existing object in the map has to be updated
  if (Get_MsgID0_From_MsgID(msg.id) == ID_Obj_3_Extended)
  {

    RCLCPP_INFO(this->get_logger(), "Received Object_3_Quality msg (0x60c)");

    /* int id = GET_Obj_3_Extended_Obj_ID(msg.data);

    object_map_list_[sensor_id][id].object_extended.obj_arellong.data =
        CALC_Obj_3_Extended_Obj_ArelLong(GET_Obj_3_Extended_Obj_ArelLong(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_extended.obj_arellat.data =
        CALC_Obj_3_Extended_Obj_ArelLat(GET_Obj_3_Extended_Obj_ArelLat(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_extended.obj_class.data =
        CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_extended.obj_orientationangle.data =
        CALC_Obj_3_Extended_Obj_OrientationAngle(GET_Obj_3_Extended_Obj_OrientationAngle(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_extended.obj_length.data =
        CALC_Obj_3_Extended_Obj_Length(GET_Obj_3_Extended_Obj_Length(msg.data), 1.0);

    object_map_list_[sensor_id][id].object_extended.obj_width.data =
        CALC_Obj_3_Extended_Obj_Width(GET_Obj_3_Extended_Obj_Width(msg.data), 1.0); */

    object_count = object_count + 1;
  };
}

// [TODO] SEND RADAR PACKET FORMAT
void radar_conti_ars408::publish_object_map(int /* sensor_id */)
{
}

} // end namespace

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// CLASS_LOADER_REGISTER_CLASS(FHAC::radar_conti_ars408, rclcpp_lifecycle::LifecycleNode)

RCLCPP_COMPONENTS_REGISTER_NODE(FHAC::radar_conti_ars408)
