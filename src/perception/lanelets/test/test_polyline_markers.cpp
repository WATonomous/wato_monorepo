// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// http://www.apache.org/licenses/LICENSE-2.0

#include <gtest/gtest.h>

#include "polyline_markers.hpp"

namespace wato::perception::lanelets
{
namespace
{

using visualization_msgs::msg::Marker;

TEST(PolylineMarkers, DrawsAllClassesInMapFrameAtTheCloudStamp)
{
  const Matrix4 ego_to_map{0, -1, 0, 100, 1, 0, 0, 200, 0, 0, 1, 2, 0, 0, 0, 1};
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 123;
  std::vector<semvecnet::postprocessing::Polyline> polylines(4);
  for (std::size_t i = 0; i < polylines.size(); ++i) {
    polylines[i].label = static_cast<int>(i);
    polylines[i].points.front() = {3.0f, 4.0f};
    polylines[i].points.back() = {10.0f, 5.0f};
  }

  const auto out = makePolylineMarkers(polylines, header, ego_to_map);
  ASSERT_EQ(out.markers.size(), 5u);
  EXPECT_EQ(out.markers.front().action, Marker::DELETEALL);
  const std::array<const char *, 4> classes{"divider", "pedestrian_crossing", "boundary", "centerline"};
  for (std::size_t i = 0; i < classes.size(); ++i) {
    const auto & marker = out.markers[i + 1];
    EXPECT_EQ(marker.header, header);
    EXPECT_EQ(marker.ns, classes[i]);
    EXPECT_EQ(marker.id, static_cast<int>(i));
    EXPECT_EQ(marker.type, Marker::LINE_STRIP);
    EXPECT_EQ(marker.action, Marker::ADD);
    EXPECT_GT(marker.scale.x, 0.0);
    EXPECT_GT(marker.color.a, 0.0f);
    ASSERT_EQ(marker.points.size(), 20u);
    EXPECT_NEAR(marker.points.front().x, 103.0, 1e-5);
    EXPECT_NEAR(marker.points.front().y, 204.943713, 1e-5);
    EXPECT_DOUBLE_EQ(marker.points.front().z, 2.0);
    EXPECT_NEAR(marker.points.back().x, 110.0, 1e-5);
    EXPECT_NEAR(marker.points.back().y, 205.943713, 1e-5);
  }
}

TEST(PolylineMarkers, PreviewDrawsSemanticMapAndRearAxleVectors)
{
  const cv::Mat rgb(300, 150, CV_8UC3, cv::Scalar(11, 22, 33));
  semvecnet::postprocessing::Polyline line{};
  line.label = 2;
  for (std::size_t i = 0; i < line.points.size(); ++i) {
    line.points[i] = {5.0f, static_cast<float>(i)};
  }
  const auto preview = renderSemVecNetPreview(rgb, {line});
  ASSERT_EQ(preview.rows, 780);
  ASSERT_EQ(preview.cols, 720);
  EXPECT_EQ(preview.at<cv::Vec3b>(50, 50), cv::Vec3b(33, 22, 11));
  // Model x=5 goes right of the ego center; forward y=19 lands above it.
  EXPECT_EQ(preview.at<cv::Vec3b>(151, 600), cv::Vec3b(0, 0, 255));
  EXPECT_NE(renderSemVecNetPreview(rgb, {}).at<cv::Vec3b>(151, 600), cv::Vec3b(0, 0, 255));
}

TEST(PolylineMarkers, ClearsPreviousSnapshotOnEmptyPrediction)
{
  const Matrix4 ego_to_map{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 124;
  const auto out = makePolylineMarkers({}, header, ego_to_map);
  ASSERT_EQ(out.markers.size(), 1u);
  EXPECT_EQ(out.markers.front().action, Marker::DELETEALL);
  EXPECT_EQ(out.markers.front().header, header);
}

}  // namespace
}  // namespace wato::perception::lanelets
