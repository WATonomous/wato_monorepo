#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "continental_pointcloud_filter.hpp"


class RadarVelocityDetectionTest : public ::testing::Test
{
public:

protected:
  filtering::ContinentalPointCloudFilter pointcloudfilter;
  filtering::ContinentalPointCloudFilter::filter_parameters parameters;
};

TEST_F(RadarVelocityDetectionTest, CheckCanMessageParsing)
{
    can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    frame_msg.header.frame_id = "can";
    receive(frame_msg.data.data(), interval_ns_)
    
    can_receive_callback(frame);
}
