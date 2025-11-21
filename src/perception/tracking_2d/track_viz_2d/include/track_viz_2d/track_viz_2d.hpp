// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRACK_VIZ_2D_HPP
#define TRACK_VIZ_2D_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include <foxglove_msgs/msg/image_annotations.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class track_viz_2d : public rclcpp::Node
{
public:
  track_viz_2d();

  static constexpr auto kDetectionsTopic = "input_detections";
  static constexpr auto kTracksTopic = "input_tracks";
  static constexpr auto kAnnotationsTopic = "output_annotations";

private:
  /**
   * @brief Initializes required parameters for visualization.
   *
   * Sets the frame, bbox colors, bbox outline width in pixels,
   * color map, and default color.
   */
  void initializeParams();

  // Callback functions
  /**
   * @brief Callback function for incoming detections.
   *
   * Saves the latest detection for use when tracks are received.
   *
   * @param msg The received detections.
   */
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  /**
   * @brief Callback function for incoming tracks.
   *
   * Saves the latest tracks and attempts to publish bounding box
   * annotations using the latest tracks and detections.
   *
   * @param msg The received tracks.
   */
  void tracksCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  // Helper functions
  /**
   * @brief Initializes a Foxglove Point2 message.
   *
   * @param x The x-coordinate.
   * @param y The y-coordinate.
   * @return foxglove_msgs::msg::Point2 The initialized Point2 message.
   */
  foxglove_msgs::msg::Point2 initPoint2(double x, double y);

  /**
   * @brief Initializes a Foxglove Color message.
   *
   * @param r The r value of RGBA.
   * @param g The g value of RGBA.
   * @param b The b value of RGBA.
   * @param a The a value of RGBA.
   * @return foxglove_msgs::msg::Color The initialized Color message.
   */
  foxglove_msgs::msg::Color initColor(double r, double g, double b, double a);

  /**
   * @brief Looks up the RGBA Foxglove Color corresponding to a color name.
   *
   * Looks up the name of the color in the unordered_map of colors.
   * If the name exists in the unordered_map, the corresonding
   * Foxglove Color is returned.
   * If the name does not exist, then a default color is returned
   * and a warning is given.
   *
   * @param color The name of the desired color.
   * @return foxglove_msgs::msg::Color The Foxglove Color corresponding to the color name.
   */
  foxglove_msgs::msg::Color colorLookup(const std::string & color);

  /**
   * @brief Creates bounding boxes from track and detection messages. .
   *
   * Initializes and populates a Foxglove PointsAnnotation message with
   * the same corner coordinates as the given Detection2D.
   *
   * @param det2d A single track or detection.
   * @param color The annotated bounding box's outline color.
   * @return foxglove_msgs::msg::PointsAnnotation The completed bounding box.
   */
  foxglove_msgs::msg::PointsAnnotation det2DToBox(
    const vision_msgs::msg::Detection2D & det2d, const foxglove_msgs::msg::Color & color);

  /**
   * @brief Attempts to publish Foxglove ImageAnnotations messages.
   *
   * If valid detections and tracks have been received, then an
   * ImageAnnotations message is populated using the PointsAnnotation
   * messages obtained from the received tracks and detections.
   *
   * Otherwise, returns immediately and a warning is given.
   */
  void tryAnnotation();

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr trks_sub_;

  // Publishers
  rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr annotations_pub_;

  // Parameters
  std::string camera_frame_;
  std::string color_dets_;
  std::string color_trks_;
  int bbox_line_width_;

  std::unordered_map<std::string, foxglove_msgs::msg::Color> color_map_;
  foxglove_msgs::msg::Color default_color_;

  vision_msgs::msg::Detection2DArray::SharedPtr latest_dets_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_trks_;
};

#endif
