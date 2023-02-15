#include <chrono>
#include <memory>

#include "ars_pointcloud_filter_node.hpp"



ARSPointCloudFilterNode::ARSPointCloudFilterNode(): Node("ars_point_cloud_filter")
{
  //default values already declared in ars_radar_params.yaml
  this->declare_parameter("filter_mode");
  this->declare_parameter("scan");
  this->declare_parameter("vrel_rad");
  this->declare_parameter("el_ang");
  this->declare_parameter("rcs0");
  this->declare_parameter("snr");
  this->declare_parameter("range");
  this->declare_parameter("az_ang0");

  raw_left_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
  "unfilteredRadarLeft", 1 ,
  std::bind(
    &ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback, this,
    std::placeholders::_1));

  raw_right_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
  "unfilteredRadarRight", 1 ,
  std::bind(
    &ARSPointCloudFilterNode::unfiltered_ars_radar_right_callback, this,
    std::placeholders::_1));

}

void ARSPointCloudFilterNode::unfiltered_ars_radar_right_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{

  RCLCPP_INFO(this->get_logger(), "Subscribing: %d\n", msg->event_id);
  // messages from unfiltered right radar topic (ars)

  std::string scan_param = this->get_parameter("scan").as_string();
  double vrel_rad_param = this->get_parameter("vrel_rad").as_double();
  double el_ang_param = this->get_parameter("el_ang").as_double();
  double rcs0_param = this->get_parameter("rcs0").as_double();
  double snr_param = this->get_parameter("snr").as_double();
  double range_param = this->get_parameter("range").as_double();
  double az_ang0_param = this->get_parameter("az_ang0").as_double();

  // Send unfiltered packets along with set parameter thresholds to the filter
  if(scan_param == "near")
  {
    if(msg->event_id == 12)
    {
      
      const radar_msgs::msg::RadarPacket test_filtered_ars = ARSPointCloudFilterNode::pointcloudfilter_.point_filter(
                                                             msg,snr_param,az_ang0_param,range_param,vrel_rad_param,el_ang_param,rcs0_param);
      
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", test_filtered_ars.event_id);

      // pointcloudfilter_.point_filter(msg,snr_param,az_ang0_param,range_param,vrel_rad_param,el_ang_param,rcs0_param);      
    }
  }

}

void ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Subscribing: %d\n", msg->event_id);
    // messages from unfiltered left radar topic (ars)
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARSPointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
