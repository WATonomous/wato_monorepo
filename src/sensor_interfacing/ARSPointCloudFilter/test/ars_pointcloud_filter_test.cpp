#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter.hpp"


class ARSPointCloudFilterFixtureTest : public ::testing::Test
{
public:
  static constexpr double DEFAULT_FILTER_PARAM = -9999.99;

  void SetUp(std::string scan_mode, double vrel_rad_param = DEFAULT_FILTER_PARAM, double el_ang_param = DEFAULT_FILTER_PARAM, 
            double rcs0_param = DEFAULT_FILTER_PARAM, double snr_param = DEFAULT_FILTER_PARAM,
            double range_param = DEFAULT_FILTER_PARAM, double az_ang0_param = DEFAULT_FILTER_PARAM)
  {
    parameters.scan_mode = scan_mode;
    parameters.vrel_rad_param = vrel_rad_param;
    parameters.el_ang_param = el_ang_param;
    parameters.rcs0_param = rcs0_param;
    parameters.snr_param = snr_param;
    parameters.range_param = range_param;
    parameters.az_ang0_param = az_ang0_param;
  }

protected:
  filtering::ARSPointCloudFilter pointcloudfilter;
  filtering::ARSPointCloudFilter::filter_parameters parameters;
};

/**
* @brief Checks if check_scan_type() correctly identifies if a packet is NEAR or FAR
*/
TEST_F(ARSPointCloudFilterFixtureTest, CheckScanType)
{
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

  // Near = 0 and Far = 1
  EXPECT_EQ(0, scan_type_0);
  EXPECT_EQ(1, scan_type_1);
}

/**
* @brief Sends 18 near scan packets with the same timestamp and checks if it returns a
         complete packet ready to be published
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendCompleteNearScanPackets)
{
  SetUp("near");

  // Near scan message
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection;

  radar_msgs::msg::RadarPacket publish_packet;

  msg->event_id = 3;
  msg->timestamp = 2;

  msg_detection.snr = 100.0;

  msg->detections.push_back(msg_detection);

  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg, parameters, publish_packet);
  }

  EXPECT_EQ(2, int(publish_packet.timestamp));
  
  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(100, publish_packet.detections[index].snr);
  }
}

/**
* @brief Sends 12 far scan packets with the same timestamp and checks if it returns a
         complete packet ready to be published
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendCompleteFarScanPackets)
{

  SetUp("far");

  // Far Scan Message
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection;

  radar_msgs::msg::RadarPacket publish_packet;

  msg->event_id = 1;
  msg->timestamp = 7;

  msg_detection.rcs0 = 5.0;

  msg->detections.push_back(msg_detection);

  for (int packet_size = 0; packet_size < 12; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg, parameters, publish_packet);
  }

  EXPECT_EQ(7, int(publish_packet.timestamp));
  
  for(int index = 0; index < 12; index++)
  {
    EXPECT_EQ(5, publish_packet.detections[index].rcs0);
  }
}

/**
* @brief Sends 18 packets that are the same (timestamp 1), and 18 packets with a new timestamp. Checks if it
         returns a complete packet ready to be published.
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendTwoDifferentScanPackets)
{
  SetUp("near");

  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection;

  radar_msgs::msg::RadarPacket publish_packet_0;
  radar_msgs::msg::RadarPacket publish_packet_1;

  // Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 7;

  msg_detection.rcs0 = 5.0;

  msg_0->detections.push_back(msg_detection);

  // Packet Data (Scan 2)
  msg_1->event_id = 4;
  msg_1->timestamp = 3;

  msg_detection.rcs0 = 20.0;

  msg_1->detections.push_back(msg_detection);

  // Scan 1 
  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_0, parameters, publish_packet_0);
  }

  EXPECT_EQ(7, int(publish_packet_0.timestamp));
  
  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(5, publish_packet_0.detections[index].rcs0);
  }

  // Scan 2
  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_1, parameters, publish_packet_1);
  }

  EXPECT_EQ(3, int(publish_packet_1.timestamp));
  
  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(20, publish_packet_1.detections[index].rcs0);
  }
}

/**
* @brief Sends incomplete packets (5 that are the same) (1 following packet that is different in timestamp)
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendIncompleteScanPackets)
{
  SetUp("near");

  // Near Scan Messages
  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection;

  radar_msgs::msg::RadarPacket publish_packet;

  // Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 7;

  msg_detection.range = 120.0; 

  msg_0->detections.push_back(msg_detection);

  // Packet Data (Scan 2)
  msg_1->event_id = 4;
  msg_1->timestamp = 3;

  msg_detection.range = 50.0;

  msg_1->detections.push_back(msg_detection);

  // Incomplete packet of near scans
  for (int packet_size = 0; packet_size < 5; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_0, parameters, publish_packet);
  }

  // Publish packet shouldn't be updated
  EXPECT_EQ(0, int(publish_packet.timestamp));

  // Scan 2 data (sending a packet of a different timestamp)
  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_1, parameters, publish_packet);
  }

  // Timestamp should be from msg_1 (msg_0 should be discarded)
  EXPECT_EQ(3, int(publish_packet.timestamp));
  
  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(50, publish_packet.detections[index].range);
  }
}

/**
* @brief Sends far scan packets when in near scan mode. Checks if it filters correctly. 
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendFarScanPacketsInNearMode)
{
  SetUp("near");

  // Near Scan Messages
  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection;

  radar_msgs::msg::RadarPacket publish_packet;

  // Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 7;

  msg_detection.range = 120.0; 

  msg_0->detections.push_back(msg_detection);

  // Packet Data (Scan 2)
  msg_1->event_id = 1;
  msg_1->timestamp = 3;

  msg_detection.range = 50.0;

  msg_1->detections.push_back(msg_detection);


  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg_0, parameters, publish_packet);
    pointcloudfilter.common_scan_filter(msg_1, parameters, publish_packet);
  }
  
  EXPECT_EQ(7, int(publish_packet.timestamp));

  for(int index = 0; index < 18; index++)
  {
    EXPECT_EQ(120, publish_packet.detections[index].range);
  }
}

/**
* @brief Checks if a packet with several detections is filtered out and if the same packet is
         returned with the correct unfiltered detection.
*/
TEST_F(ARSPointCloudFilterFixtureTest, CheckIfDetectionsFiltered)
{
  SetUp("near", DEFAULT_FILTER_PARAM, DEFAULT_FILTER_PARAM, 70.0, 100.0, 40.0, DEFAULT_FILTER_PARAM);

  // Near Scan Message
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;
  radar_msgs::msg::RadarDetection msg_detection_2;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet;

  // Packet Data (Scan 1)
  msg->event_id = 3;
  msg->timestamp = 7;

  // Sending message with no detections
  for(int packet_size = 0; packet_size < 17; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg, parameters, publish_packet);
  }

  // msg snr < snr threshold
  msg_detection_0.range = 120.0; 
  msg_detection_0.rcs0 = 75.0;
  msg_detection_0.snr = 80.0;

  msg->detections.push_back(msg_detection_0);

  // Passes all the filter parameters
  msg_detection_1.range = 50.0; 
  msg_detection_1.rcs0 = 90.0;
  msg_detection_1.snr = 200.0;

  msg->detections.push_back(msg_detection_1);

  // msg range < range threshold & msg snr < snr threshold
  msg_detection_2.range = 10.0; 
  msg_detection_2.rcs0 = 90.0;
  msg_detection_2.snr = 50.0;

  msg->detections.push_back(msg_detection_2);

  for (int packet_size = 0; packet_size < 1; packet_size++)
  {
    pointcloudfilter.common_scan_filter(msg, parameters, publish_packet);
  }
  
  EXPECT_EQ(7, int(publish_packet.timestamp));

  // Returns a published packet with one detection going through
  EXPECT_EQ(1, int(publish_packet.detections.size()));
  EXPECT_EQ(50, int(publish_packet.detections[0].range));
  EXPECT_EQ(90, int(publish_packet.detections[0].rcs0));
  EXPECT_EQ(200, int(publish_packet.detections[0].snr));
}

// Double Buffer Algorithm Test

/**
* @brief Send 18 Near Scan Packets and 12 Far Scan Packets 
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendCompleteNearAndFarPackets)
{
  SetUp("nearfar");

  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet_;

  // Near Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 5;

  msg_detection_0.range = 80.0; 

  msg_0->detections.push_back(msg_detection_0);

  // Far Packet Data (Scan 1)
  msg_1->event_id = 1;
  msg_1->timestamp = 6;

  msg_detection_1.range = 90.0;

  msg_1->detections.push_back(msg_detection_1);

  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_0, parameters, publish_packet_);
  }

  // Shouldn't be published at this point
  EXPECT_EQ(0, int(publish_packet_.timestamp));
  EXPECT_EQ(0, publish_packet_.detections.size());

  for (int packet_size = 0; packet_size < 12; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_1, parameters, publish_packet_);
  }

  EXPECT_EQ(6, int(publish_packet_.timestamp));
  EXPECT_EQ(30, publish_packet_.detections.size());
  EXPECT_EQ(80, publish_packet_.detections[0].range);
  EXPECT_EQ(90, publish_packet_.detections[19].range);
}

/**
* @brief 18 Near scan (scan 1), 10 far scan (scan 1), 3 Near (scan 2), 2 far (scan 1) (SCAN 1 SHOULD BE PUBLISHED), 
         15 Near (scan 2), 12 Far (scan2) (SCAN 2 GETS PUBLISHED HERE)
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendMultipleScanPackets)
{
  SetUp("nearfar");

  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_2 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_3 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet_1;
  radar_msgs::msg::RadarPacket publish_packet_2;

  // Near Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 5;

  msg_detection_0.range = 80.0; 

  msg_0->detections.push_back(msg_detection_0);

  // Far Packet Data (Scan 1)
  msg_1->event_id = 1;
  msg_1->timestamp = 6;

  msg_detection_1.range = 120.0; 

  msg_1->detections.push_back(msg_detection_1);

  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_0, parameters, publish_packet_1);
  }

  // Scan 1 data shouldn't be published at this point
  EXPECT_EQ(0, int(publish_packet_1.timestamp));
  EXPECT_EQ(0, publish_packet_1.detections.size());

  for (int packet_size = 0; packet_size < 10; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_1, parameters, publish_packet_1);
  }

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"NEW CHECKPOINT 1 \n ");

  // Scan 1 data shouldn't be published at this point
  EXPECT_EQ(0, int(publish_packet_1.timestamp));
  EXPECT_EQ(0, publish_packet_1.detections.size());

  // Near Packet Data (Scan 2)
  msg_2->event_id = 3;
  msg_2->timestamp = 8;

  msg_detection_0.range = 150.0; 

  msg_2->detections.push_back(msg_detection_0);

  for (int packet_size = 0; packet_size < 3; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_2, parameters, publish_packet_2);
  }
  EXPECT_EQ(0, int(publish_packet_2.timestamp));
  EXPECT_EQ(0, publish_packet_2.detections.size());

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"NEW CHECKPOINT 2 \n ");

  for (int packet_size = 0; packet_size < 2; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_1, parameters, publish_packet_1);
  }

  // Packet 1 should get published here since scan 1 data is full and all scan states should be reset
  EXPECT_EQ(6, int(publish_packet_1.timestamp));
  EXPECT_EQ(30, publish_packet_1.detections.size());

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"NEW CHECKPOINT 3 \n ");

  // Sending data from scan 2
  for (int packet_size = 0; packet_size < 15; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_0, parameters, publish_packet_2);
  }

  EXPECT_EQ(0, int(publish_packet_2.timestamp));
  EXPECT_EQ(0, publish_packet_2.detections.size());

  // Far Packet Data (Scan 2)
  msg_3->event_id = 1;
  msg_3->timestamp = 9;

  msg_detection_1.range = 210.0; 

  msg_3->detections.push_back(msg_detection_1);

  for (int packet_size = 0; packet_size < 12; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_3, parameters, publish_packet_2);
  }

  EXPECT_EQ(9, int(publish_packet_2.timestamp));
  EXPECT_EQ(150, publish_packet_2.detections[0].range);
  EXPECT_EQ(210, publish_packet_2.detections[19].range);
  EXPECT_EQ(30, publish_packet_2.detections.size());
}

/**
* @brief Send 5 Near (scan 1), Send 12 far (scan 1)
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendIncompleteNearPackets)
{
  SetUp("nearfar");

  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_2 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_3 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;

  // Fake Publish packets
  radar_msgs::msg::RadarPacket publish_packet;

 // Near Packet Data (Scan 1)
  msg_0->event_id = 3;
  msg_0->timestamp = 5;

  msg_detection_0.range = 80.0; 

  msg_0->detections.push_back(msg_detection_0);

  // Far Packet Data (Scan 1)
  msg_1->event_id = 1;
  msg_1->timestamp = 6;

  msg_detection_1.range = 120.0; 

  msg_1->detections.push_back(msg_detection_1);

  for (int packet_size = 0; packet_size < 5; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_0, parameters, publish_packet);
  }

  EXPECT_EQ(0, int(publish_packet.timestamp));
  EXPECT_EQ(0, publish_packet.detections.size());

  for (int packet_size = 0; packet_size < 12; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_1, parameters, publish_packet);
  }

  // Special Case data should be discarded at this point
  EXPECT_EQ(0, int(publish_packet.timestamp));
  EXPECT_EQ(0, publish_packet.detections.size());

  // Sending a full packet to validate that previous data was actually discarded

  // Near Packet Data (Scan 2)
  msg_2->event_id = 3;
  msg_2->timestamp = 7;

  msg_detection_0.range = 160.0; 

  msg_2->detections.push_back(msg_detection_0);

  // Far Packet Data (Scan 2)
  msg_3->event_id = 1;
  msg_3->timestamp = 8;

  msg_detection_1.range = 200.0; 

  msg_3->detections.push_back(msg_detection_1);

  for(int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_2, parameters, publish_packet);
  }

  for(int packet_size = 0; packet_size < 12; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_3, parameters, publish_packet);
  }

  EXPECT_EQ(8, int(publish_packet.timestamp));
  EXPECT_EQ(30, publish_packet.detections.size());
  EXPECT_EQ(160, publish_packet.detections[0].range);
  EXPECT_EQ(200, publish_packet.detections[19].range);
}

/**
* @brief Send 7 far (scan 1) (NEAR SCANS WERE IGNORED), Send 1 Near (scan 2), then check if the 7 far is published
*/
TEST_F(ARSPointCloudFilterFixtureTest, SendIncompleteFarPackets)
{
  SetUp("nearfar");

  auto msg_0 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_1 = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_2 = std::make_shared<radar_msgs::msg::RadarPacket>();

  radar_msgs::msg::RadarDetection msg_detection_0;
  radar_msgs::msg::RadarDetection msg_detection_1;
  radar_msgs::msg::RadarDetection msg_detection_2;

  // Fake Publish Packet
  radar_msgs::msg::RadarPacket publish_packet;

  // Far Packet Data (Scan 1)
  msg_0->event_id = 1;
  msg_0->timestamp = 6;

  msg_detection_0.range = 80.0; 

  msg_0->detections.push_back(msg_detection_0);

  // Near Packet Data (Scan 2)
  msg_1->event_id = 4;
  msg_1->timestamp = 7;

  msg_detection_1.range = 120.0; 

  msg_1->detections.push_back(msg_detection_1);

  // Far Packet Data (Scan 2)
  msg_2->event_id = 1;
  msg_2->timestamp = 8;

  msg_detection_2.range = 180.0; 

  msg_2->detections.push_back(msg_detection_2);

  // Sending far packet data from scan 1
  for (int packet_size = 0; packet_size < 7; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_0, parameters, publish_packet);
  }

  EXPECT_EQ(0, int(publish_packet.timestamp));
  EXPECT_EQ(0, publish_packet.detections.size());

  // Sending near packet data from scan 2
  for (int packet_size = 0; packet_size < 18; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_1, parameters, publish_packet);
  }

  EXPECT_EQ(0, int(publish_packet.timestamp));
  EXPECT_EQ(0, publish_packet.detections.size());

  // Sending far packet data from scan 2
  for (int packet_size = 0; packet_size < 12; packet_size++)
  {
    pointcloudfilter.near_far_scan_filter(msg_2, parameters, publish_packet);
  }

  EXPECT_EQ(8, int(publish_packet.timestamp));
  EXPECT_EQ(30, publish_packet.detections.size());
}


// Unit test cases (Common Scan Filter)
// 1. Send 18 near scan packets and check if it returns a complete 18 packet ready to be published (done)
// 2. Send 12 far scan packets and check if it returns a complete 12 packet ready to be published (done)
// 3. Send 18 that are the same (different timestamp), and 18 that are different (different timestamp) (done)
// 4. Send incomplete packets (5 that are the same) (1 that is different in timestamp) (done)
// 5. Send 19 near scan packets and see if it behaves correctly (is not required) (done)
// 6. Send far scan packets when in near scan mode, does it filter correctly (done)
// 7. Check if it filters out all the packets and returns an empty packet (pass thresholds into pointfilter) (done)

// Unit Test Cases (Double Buffer Algorithm)
// 1. Send 18 Near Scan Packets and 12 Far Scan Packets (done)
// (It shouldn't publish after 18 packets. It should publish together when there is also complete 12 far scan packets)
// 2. 18 Near scan (scan1), 10 far scan (scan1), 3 Near (scan 2), 2 far (scan 1) (SCAN 1 SHOULD BE PUBLISHED), 15 Near (scan 2), 12 Far (scan2) (SCAN 2 GETS PUBLISHED HERE) (done)
// 3. Send 7 far (scan 1) (NEAR SCANS WERE IGNORED), Send 1 Near (Scan 2), Send 1 Far (Scan 2)then check if the 7 far is published 
// 4. Send 5 Near (scan 1), Send 12 far (scan 1) (done)