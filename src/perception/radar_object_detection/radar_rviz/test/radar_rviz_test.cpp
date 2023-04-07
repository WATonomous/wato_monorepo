#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "radar_rviz.hpp"
#include "radar_rviz_node.hpp"

class RadarRvizProcessorNodeFixtureTest : public ::testing::Test
{
public:
  void SetUp() {}
};

// TEST_F(RadarRvizProcessorNodeFixtureTest, CheckNodeCallback)
// {
//   RadarRvizProcessorNode test_node;

//   // Near Scan message
//   auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();
//   radar_msgs::msg::RadarDetection msg_detection;

//   msg->event_id = 3;
//   msg->timestamp = 2;

//   msg_detection.pos_x = 100.0;
//   msg_detection.pos_y = 200.0;
//   msg_detection.pos_z = 240.0;
//   msg_detection.rcs0 = 90.0;
//   msg->detections.push_back(msg_detection);

//   msg_detection.pos_x = 10.0;
//   msg_detection.pos_y = 80.0;
//   msg_detection.pos_z = 90.0;
//   msg_detection.rcs0 = 10.0;
//   msg->detections.push_back(msg_detection);

//   // test_node.process_radar_data_callback(msg);
//   // while(true)
//   // {
//   //   test_node.process_radar_data_callback(msg);
//   // }
// }
