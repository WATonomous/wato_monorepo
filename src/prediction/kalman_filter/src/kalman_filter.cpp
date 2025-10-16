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

#include "kalman_filter/kalman_filter.hpp"

#include <rclcpp/rclcpp.hpp>

class KalmanFilterNode : public rclcpp::Node
{
public:
  KalmanFilterNode()
  : Node("kalman_filter_node")
  {
    RCLCPP_INFO(this->get_logger(), "Kalman Filter Node started");

    // TODO(wato): Add your subscribers, publishers, timers, etc. here
    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KalmanFilterNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // TODO(wato): Implement your Kalman filter logic here
    RCLCPP_DEBUG(this->get_logger(), "Kalman filter running...");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  KalmanFilter kalman_filter_;  // Use your KalmanFilter class here
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
