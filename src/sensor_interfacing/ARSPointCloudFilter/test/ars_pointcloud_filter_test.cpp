#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter.hpp"

// // The fixture for testing class ARSPointCloudFilterTest
// class ARSPointCloudFilterTest : public ::testing::Test {
//  protected:

//   ARSPointCloudFilterTest() {
//     filtering::ARSPointCloudFilter::filter_parameters parameters;
//     parameters.scan_mode = "near";
//     parameters.vrel_rad_param = -9999.99;
//     parameters.el_ang_param = -9999.99;
//     parameters.rcs0_param = -9999.99;
//     parameters.snr_param = -9999.99;
//     parameters.range_param = -9999.99;
//     parameters.az_ang0_param = -9999.99;
//   }

//   void SetUp() override {
//      // Code here will be called immediately after the constructor (right
//      // before each test).
//   }

//   void TearDown() override {
//      // Code here will be called immediately after each test (right
//      // before the destructor).
//   }

// };


// Checks if check_scan_type() correctly identifies if a packet is NEAR or FAR
TEST(ARSPointCloudFilterTest, CheckScanType)
{
  filtering::ARSPointCloudFilter pointcloudfilter;

  // Near Scan message
  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();

  msg_0->event_id = 3;
  msg_0->timestamp = 2;

  // Far Scan message
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  msg_1->event_id = 1;
  msg_1->timestamp = 2;

  auto scan_type_0 = pointcloudfilter.check_scan_type(msg_0);
  auto scan_type_1 = pointcloudfilter.check_scan_type(msg_1);

  EXPECT_EQ(0, scan_type_0);
  EXPECT_EQ(1, scan_type_1);
}


// Send 18 near scan packets with the same timestamp and check if it returns a complete packet ready to be published
TEST(ARSPointCloudFilterTest, CommonScanFilter_01)
{
  // ONLY NEAR SCAN DATA
  filtering::ARSPointCloudFilter pointcloudfilter;
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection;

  // Fake Publish packet
  radar_msgs::msg::RadarPacket publish_packet;

  filtering::ARSPointCloudFilter::filter_parameters parameters;
  parameters.scan_mode = "near";
  parameters.vrel_rad_param = -9999.99;
  parameters.el_ang_param = -9999.99;
  parameters.rcs0_param = -9999.99;
  parameters.snr_param = -9999.99;
  parameters.range_param = -9999.99;
  parameters.az_ang0_param = -9999.99;

  // Packet Data
  msg->event_id = 3;
  msg->timestamp = 2;

  msg_detection.snr = 100.0;

  msg->detections.push_back(msg_detection);

  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg, parameters, publish_packet);
  }

  EXPECT_EQ(2, publish_packet.timestamp);
  
  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(100, publish_packet.detections[index].snr);
  }
  
}

// Send 12 far scan packets with the same timestamp and check if it returns a complete packet ready to be published
TEST(ARSPointCloudFilterTest, CommonScanFilter_02)
{
  // ONLY FAR SCAN DATA
  filtering::ARSPointCloudFilter pointcloudfilter;
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection;

  // Fake Publish packet
  radar_msgs::msg::RadarPacket publish_packet;

  // Fake parameters
  filtering::ARSPointCloudFilter::filter_parameters parameters;
  parameters.scan_mode = "far";
  parameters.vrel_rad_param = -9999.99;
  parameters.el_ang_param = -9999.99;
  parameters.rcs0_param = -9999.99;
  parameters.snr_param = -9999.99;
  parameters.range_param = -9999.99;
  parameters.az_ang0_param = -9999.99;

  // Packet Data
  msg->event_id = 1;
  msg->timestamp = 7;

  msg_detection.rcs0 = 5.0;

  msg->detections.push_back(msg_detection);

  for (int packet_size = 0; packet_size < 12; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg, parameters, publish_packet);
  }

  EXPECT_EQ(7, publish_packet.timestamp);
  
  for(int index = 0; index < 12; index++)
  {
    EXPECT_EQ(5, publish_packet.detections[index].rcs0);
  }
}

// Send 18 that are the same (different timestamp), and 18 that are different (different timestamp) 
TEST(ARSPointCloudFilterTest, CommonScanFilter_03)
{
  // Near Scan data
  filtering::ARSPointCloudFilter pointcloudfilter;
  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet_0;
  radar_msgs::msg::RadarPacket publish_packet_1;

  // Fake parameters
  filtering::ARSPointCloudFilter::filter_parameters parameters;
  parameters.scan_mode = "near";
  parameters.vrel_rad_param = -9999.99;
  parameters.el_ang_param = -9999.99;
  parameters.rcs0_param = -9999.99;
  parameters.snr_param = -9999.99;
  parameters.range_param = -9999.99;
  parameters.az_ang0_param = -9999.99;

  // Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 7;


  msg_detection_0.rcs0 = 5.0;

  msg_0->detections.push_back(msg_detection_0);

  // Packet Data (Scan 2)
  msg_1->event_id = 4;
  msg_1->timestamp = 3;

  msg_detection_1.rcs0 = 20.0;

  msg_1->detections.push_back(msg_detection_1);

  // Scan 1 
  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_0, parameters, publish_packet_0);
  }

  EXPECT_EQ(7, publish_packet_0.timestamp);
  
  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(5, publish_packet_0.detections[index].rcs0);
  }

  // Scan 2
  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_1, parameters, publish_packet_1);
  }

  EXPECT_EQ(3, publish_packet_1.timestamp);
  
  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(20, publish_packet_1.detections[index].rcs0);
  }
  
}

// Send incomplete packets (5 that are the same) (18 that is different in timestamp)
TEST(ARSPointCloudFilterTest, CommonScanFilter_04)
{
  // Near Scan data
  filtering::ARSPointCloudFilter pointcloudfilter;
  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet_;

  // Fake parameters
  filtering::ARSPointCloudFilter::filter_parameters parameters;
  parameters.scan_mode = "near";
  parameters.vrel_rad_param = -9999.99;
  parameters.el_ang_param = -9999.99;
  parameters.rcs0_param = -9999.99;
  parameters.snr_param = -9999.99;
  parameters.range_param = -9999.99;
  parameters.az_ang0_param = -9999.99;

  // Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 7;

  msg_detection_0.range = 120.0; 

  msg_0->detections.push_back(msg_detection_0);

  // Packet Data (Scan 2)
  msg_1->event_id = 4;
  msg_1->timestamp = 3;

  msg_detection_1.range = 50.0;

  msg_1->detections.push_back(msg_detection_1);

  // Incomplete packet of near scans
  for (int packet_size = 0; packet_size < 5; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_0, parameters, publish_packet_);
  }

  // Publish packet shouldn't be updated
  EXPECT_EQ(0, publish_packet_.timestamp);


  // Scan 2 data (sending a packet of a different timestamp)
  for (int packet_size = 0; packet_size < 1; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_1, parameters, publish_packet_);
  }

  // Timestamp should be from msg_0
  EXPECT_EQ(7, publish_packet_.timestamp);
  
  for(int index = 0; index < 5; index++)
  {
    EXPECT_EQ(120, publish_packet_.detections[index].range);
  }
}

// Send far scan packets when in near scan mode. Check if it filters correctly. 
TEST(ARSPointCloudFilterTest, CommonScanFilter_05)
{
  // Near Scan data
  filtering::ARSPointCloudFilter pointcloudfilter;
  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet_;

  // Fake parameters
  filtering::ARSPointCloudFilter::filter_parameters parameters;
  parameters.scan_mode = "near";
  parameters.vrel_rad_param = -9999.99;
  parameters.el_ang_param = -9999.99;
  parameters.rcs0_param = -9999.99;
  parameters.snr_param = -9999.99;
  parameters.range_param = -9999.99;
  parameters.az_ang0_param = -9999.99;

  // Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 7;

  msg_detection_0.range = 120.0; 

  msg_0->detections.push_back(msg_detection_0);

  // Packet Data (Scan 2)
  msg_1->event_id = 1;
  msg_1->timestamp = 3;

  msg_detection_1.range = 50.0;

  msg_1->detections.push_back(msg_detection_1);


  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_0, parameters, publish_packet_);
    pointcloudfilter.common_scan_filter(msg_1, parameters, publish_packet_);
  }
  
  EXPECT_EQ(7, publish_packet_.timestamp);

  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(120, publish_packet_.detections[index].range);
  }

}

// Check if it filters out all the packets and returns an empty packet (pass thresholds into pointfilter)
TEST(ARSPointCloudFilterTest, CommonScanFilter_06)
{
  // Near Scan data
  filtering::ARSPointCloudFilter pointcloudfilter;
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;
  radar_msgs::msg::RadarDetection msg_detection_2;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet_;

  // Fake parameters
  filtering::ARSPointCloudFilter::filter_parameters parameters;
  parameters.scan_mode = "near";
  parameters.vrel_rad_param = -9999.99;
  parameters.el_ang_param = -9999.99;
  parameters.rcs0_param = 70.0;
  parameters.snr_param = 100.0;
  parameters.range_param = 40.0;
  parameters.az_ang0_param = -9999.99;

  // Packet Data (Scan 1)
  msg->event_id = 3;
  msg->timestamp = 7;

  msg_detection_0.range = 120.0; 
  msg_detection_0.rcs0 = 75.0;
  msg_detection_0.snr = 80.0;

  msg->detections.push_back(msg_detection_0);

  msg_detection_1.range = 50.0; 
  msg_detection_1.rcs0 = 60.0;
  msg_detection_1.snr = 200.0;

  msg->detections.push_back(msg_detection_1);

  msg_detection_2.range = 10.0; 
  msg_detection_2.rcs0 = 90.0;
  msg_detection_2.snr = 50.0;

  msg->detections.push_back(msg_detection_2);


  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg, parameters, publish_packet_);
  }
  
  EXPECT_EQ(7, publish_packet_.timestamp);

  // Return published packet with empty detections
  EXPECT_EQ(0, publish_packet_.detections.size());

}
// Unit test cases 
// 1. Send 18 near scan packets and check if it returns a complete 18 packet ready to be published (done)
// 2. Send 12 far scan packets and check if it returns a complete 12 packet ready to be published (done)
// 3. Send 18 that are the same (different timestamp), and 18 that are different (different timestamp) (done)
// 4. Send incomplete packets (5 that are the same) (1 that is different in timestamp) (done)
// 5. Send 19 near scan packets and see if it behaves correctly (Is not required) (done)
// 6. Send far scan packets when in near scan mode, does it filter correctly (done)
// 7. Check if it filters out all the packets and returns an empty packet (pass thresholds into pointfilter) (done)




// TEST(ARSPointCloudFilterTest, PointCloudFilter)
// {
//   filtering::ARSPointCloudFilter pointcloudfilter;
//   auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();
//   auto msg_detection = std::make_shared<radar_msgs::msg::RadarDetection>();

//   msg->event_id = 3;
//   msg->timestamp = 2;
//   msg->measurement_counter = 1;
//   msg->vambig = 3.0;
//   msg->center_frequency = 6.0;

//   // Fake detection Data 1
//   msg_detection->vrel_rad = 100.0;
//   msg_detection->az_ang0 = 60.0;
//   msg_detection->el_ang = 20.0;
//   msg_detection->rcs0 = 5.0;
//   msg_detection->snr = 100.0;
//   msg_detection->range = 90.0;

//   msg->detections.push_back(*msg_detection);

//   // Fake detection Data 2
//   msg_detection->vrel_rad = 100.0;
//   msg_detection->az_ang0 = 20.0;
//   msg_detection->el_ang = 20.0;
//   msg_detection->rcs0 = 5.0;
//   msg_detection->snr = 300.0;
//   msg_detection->range = 90.0;

//   msg->detections.push_back(*msg_detection);
  
//   auto test_packet = pointcloudfilter.point_filter(msg, 200, -9999.99, -9999.99, -9999.99, -9999.99, -9999.99);
  
//   // Fake detection data 1 should be removed and fake detection data 2 should be at index 0
//   EXPECT_EQ(300, test_packet.detections[0].snr);

//   test_packet = pointcloudfilter.point_filter(msg, 100, 40, -9999.99, -9999.99, -9999.99, -9999.99);
//   // Fake detection data 1 should be left
//   EXPECT_EQ(100, test_packet.detections[0].snr);
//   EXPECT_EQ(60, test_packet.detections[0].az_ang0);

//   // Fake detection data 2 should not exist, so all values should return 0
//   EXPECT_EQ(0, test_packet.detections[1].snr);
//   EXPECT_EQ(0, test_packet.detections[1].az_ang0);
// }
