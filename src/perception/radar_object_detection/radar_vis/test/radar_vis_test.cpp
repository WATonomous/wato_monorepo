#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "radar_vis.hpp"
#include "radar_vis_node.hpp"


TEST(RadarVisNodeTest, CheckConvertPacketToPointCloud)
{
  visualization::RadarVis test_vis;
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();
  radar_msgs::msg::RadarDetection msg_detection;
  sensor_msgs::msg::PointCloud2 test_point_cloud;

  msg->event_id = 3;
  msg->timestamp = 2;

  msg_detection.pos_x = 1.0;
  msg_detection.pos_y = 1.0;
  msg_detection.pos_z = 1.0;
  msg_detection.rcs0 = 20.0;
  msg->detections.push_back(msg_detection);

  test_point_cloud = test_vis.convert_packet_to_pointcloud(msg);

  EXPECT_EQ(test_point_cloud.header.frame_id, "radar_fixed");
  EXPECT_EQ(test_point_cloud.height, 1);
  EXPECT_EQ(test_point_cloud.width, 1);
  EXPECT_EQ(test_point_cloud.is_bigendian, false);
  EXPECT_EQ(test_point_cloud.point_step, 16);
  EXPECT_EQ(test_point_cloud.row_step, 16);
  EXPECT_EQ(test_point_cloud.is_dense, true);

  EXPECT_EQ(test_point_cloud.fields[0].name, "x");
  EXPECT_EQ(test_point_cloud.fields[0].offset, 0);

  EXPECT_EQ(test_point_cloud.fields[1].name, "y");
  EXPECT_EQ(test_point_cloud.fields[1].offset, 4);

  EXPECT_EQ(test_point_cloud.fields[2].name, "z");
  EXPECT_EQ(test_point_cloud.fields[2].offset, 8);

  EXPECT_EQ(test_point_cloud.fields[3].name, "intensity");
  EXPECT_EQ(test_point_cloud.fields[3].offset, 12);

  // Little Endian Conversion (32 bit float into 4 bytes; each 4 bytes represents one point field)

  // Position X
  EXPECT_EQ(test_point_cloud.data[0], 0);
  EXPECT_EQ(test_point_cloud.data[1], 0);
  EXPECT_EQ(test_point_cloud.data[2], 128);
  EXPECT_EQ(test_point_cloud.data[3], 63);

  // Position Y
  EXPECT_EQ(test_point_cloud.data[4], 0);
  EXPECT_EQ(test_point_cloud.data[5], 0);
  EXPECT_EQ(test_point_cloud.data[6], 128);
  EXPECT_EQ(test_point_cloud.data[7], 63);

  // Position Z
  EXPECT_EQ(test_point_cloud.data[8], 0);
  EXPECT_EQ(test_point_cloud.data[9], 0);
  EXPECT_EQ(test_point_cloud.data[10], 128);
  EXPECT_EQ(test_point_cloud.data[11], 63);

  // Intensity
  EXPECT_EQ(test_point_cloud.data[12], 0);
  EXPECT_EQ(test_point_cloud.data[13], 0);
  EXPECT_EQ(test_point_cloud.data[14], 112);
  EXPECT_EQ(test_point_cloud.data[15], 66);
}
