// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// http://www.apache.org/licenses/LICENSE-2.0

#include "polyline_markers.hpp"

#include <cmath>
#include <cstdint>

#include <opencv2/imgproc.hpp>
#include <semvecnet/base-config.hpp>

namespace wato::perception::lanelets
{
namespace
{

struct Style
{
  const char * ns;
  int r, g, b;
};
constexpr Style kStyles[] = {
  {"divider", 255, 165, 0},
  {"pedestrian_crossing", 0, 80, 255},
  {"boundary", 255, 0, 0},
  {"centerline", 0, 200, 0},
  {"unknown", 255, 255, 255},
};

const Style & styleFor(int label)
{
  return kStyles[label >= 0 && label < 4 ? label : 4];
}

std::array<float, 2> toRearAxle(const std::array<float, 2> & point)
{
  // Match CUDA-SemVecNet/src/main.cpp: model right/forward -> rear axle forward/left.
  constexpr float lidar_forward_offset = 0.943713f;
  return {point[1] + lidar_forward_offset, -point[0]};
}

}  // namespace

visualization_msgs::msg::MarkerArray makePolylineMarkers(
  const std::vector<semvecnet::postprocessing::Polyline> & polylines,
  const std_msgs::msg::Header & map_header,
  const Matrix4 & ego_to_map)
{
  using visualization_msgs::msg::Marker;

  visualization_msgs::msg::MarkerArray out;
  out.markers.reserve(1 + polylines.size());
  // Replace the previous snapshot on this dedicated topic, even on an empty prediction.
  auto & clear = out.markers.emplace_back();
  clear.header = map_header;
  clear.action = Marker::DELETEALL;

  // Invert the ego-centric crop using the same planar heading as SemVecNet's renderer.
  const auto yaw = std::atan2(ego_to_map[4], ego_to_map[5]);
  const auto cos_yaw = std::cos(yaw);
  const auto sin_yaw = std::sin(yaw);
  for (const auto & polyline : polylines) {
    const auto & style = styleFor(polyline.label);
    auto & marker = out.markers.emplace_back();
    marker.header = map_header;
    marker.ns = style.ns;
    marker.id = static_cast<std::int32_t>(out.markers.size() - 2);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.12;
    marker.color.r = style.r / 255.0f;
    marker.color.g = style.g / 255.0f;
    marker.color.b = style.b / 255.0f;
    marker.color.a = 1.0f;
    marker.points.reserve(polyline.points.size());
    for (const auto & model_point : polyline.points) {
      const auto [x, y] = toRearAxle(model_point);
      auto & point = marker.points.emplace_back();
      point.x = ego_to_map[3] + cos_yaw * x - sin_yaw * y;
      point.y = ego_to_map[7] + sin_yaw * x + cos_yaw * y;
      point.z = ego_to_map[11];
    }
  }
  return out;
}

cv::Mat renderSemVecNetPreview(
  const cv::Mat & semantic_rgb, const std::vector<semvecnet::postprocessing::Polyline> & polylines)
{
  constexpr double scale = 2.4;
  constexpr int header = 30;
  constexpr int footer = 30;
  const int w = cvRound(semantic_rgb.cols * scale);
  const int h = cvRound(semantic_rgb.rows * scale);
  cv::Mat image(h + header + footer, 2 * w, CV_8UC3, cv::Scalar(24, 24, 24));
  auto map_panel = image(cv::Rect(0, header, w, h));
  auto vectors = image(cv::Rect(w, header, w, h));
  cv::Mat semantic_bgr;
  cv::cvtColor(semantic_rgb, semantic_bgr, cv::COLOR_RGB2BGR);
  cv::resize(semantic_bgr, map_panel, map_panel.size(), 0, 0, cv::INTER_NEAREST);
  const auto pixel = [w, h](float forward, float left) {
    return cv::Point(
      cvRound(w / 2.0 - left * scale / semvecnet::kBevResolution),
      cvRound(h / 2.0 - forward * scale / semvecnet::kBevResolution));
  };
  const cv::Scalar grid(45, 45, 45), white(255, 255, 255);
  for (int m = -30; m <= 30; m += 10) {
    const int y = pixel(m, 0).y;
    cv::line(vectors, {0, y}, {w - 1, y}, grid);
  }
  for (int m : {-10, 0, 10}) {
    const int x = pixel(0, m).x;
    cv::line(vectors, {x, 0}, {x, h - 1}, grid);
  }
  for (const auto & line : polylines) {
    std::vector<cv::Point> points;
    points.reserve(line.points.size());
    for (const auto & point : line.points) {
      const auto [forward, left] = toRearAxle(point);
      points.push_back(pixel(forward, left));
    }
    const auto & style = styleFor(line.label);
    const cv::Scalar bgr(style.b, style.g, style.r);
    cv::polylines(vectors, points, false, bgr, 2, cv::LINE_AA);
    cv::circle(vectors, points.back(), 3, bgr, cv::FILLED, cv::LINE_AA);
  }
  cv::rectangle(vectors, pixel(3.6f, 0.95f), pixel(-0.9f, -0.95f), white);
  cv::circle(vectors, pixel(0, 0), 2, white, cv::FILLED);
  const auto title = [&](const std::string & text, int x, int y) {
    cv::putText(image, text, {x, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, white, 1, cv::LINE_AA);
  };
  title("SemVecNet input: semantic map", 8, 20);
  title("SemVecNet output: " + std::to_string(polylines.size()) + " polylines", w + 8, 20);
  for (int i = 0; i < 4; ++i) {
    const auto & style = styleFor(i);
    const int x = i * image.cols / 4 + 8;
    cv::line(image, {x, h + header + 15}, {x + 20, h + header + 15}, {double(style.b), double(style.g), double(style.r)}, 2);
    title(style.ns, x + 26, h + header + 20);
  }
  return image;
}

}  // namespace wato::perception::lanelets
