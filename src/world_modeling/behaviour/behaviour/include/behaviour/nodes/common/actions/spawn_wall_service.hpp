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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__SPAWN_WALL_SERVICE_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__SPAWN_WALL_SERVICE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <behaviortree_ros2/bt_service_node.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/utils.hpp"

#include "costmap_msgs/srv/spawn_wall.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace behaviour
{
    /**
     * @class SpawnWallService
     * @brief BT node to request spawning a virtual wall (costmap layer).
     *
     * Frame handling:
     *  - Input pose MUST have a valid header.frame_id that exists in TF.
     *  - This node transforms pose -> base_frame (default "base_link") before calling the service.
     *
     * Required inputs:
     *  - pose: geometry_msgs::msg::PoseStamped::SharedPtr
     *  - tf_buffer: std::shared_ptr<tf2_ros::Buffer>
     *
     * Optional inputs:
     *  - base_frame: target frame for the wall pose (default "base_link")
     *  - width, length: wall dimensions (defaults provided)
     *
     * Outputs:
     *  - wall_id
     *  - error_message (set only on failure paths)
     */
    class SpawnWallService : public BT::RosServiceNode<costmap_msgs::srv::SpawnWall>
    {
    public:
        SpawnWallService(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosServiceNode<costmap_msgs::srv::SpawnWall>(name, conf, params)
        {
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose"),
                BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"),

                BT::InputPort<std::string>("base_frame"),
                BT::InputPort<double>("width"),
                BT::InputPort<double>("length"),

                BT::OutputPort<int32_t>("wall_id"),
                BT::OutputPort<std::string>("error_message"),
            });
        }

        bool setRequest(Request::SharedPtr &request) override
        {
            // Read inputs
            auto input_pose = ports::tryGetPtr<geometry_msgs::msg::PoseStamped>(*this, "pose");
            if (!input_pose)
            {
                RCLCPP_ERROR(logger(), "[%s] missing input port: pose", name().c_str());
                setOutput("error_message", "missing_port:pose");
                return false;
            }

            auto tf_buffer = ports::tryGetPtr<tf2_ros::Buffer>(*this, "tf_buffer");
            if (!tf_buffer)
            {
                RCLCPP_ERROR(logger(), "[%s] missing input port: tf_buffer", name().c_str());
                setOutput("error_message", "missing_port:tf_buffer");
                return false;
            }

            const std::string base_frame =
                ports::tryGet<std::string>(*this, "base_frame").value_or("base_link");

            const double width = ports::tryGet<double>(*this, "width").value_or(5.0);
            const double length = ports::tryGet<double>(*this, "length").value_or(1.0);

            if (input_pose->header.frame_id.empty())
            {
                RCLCPP_ERROR(logger(), "[%s] input_pose->header.frame_id is empty", name().c_str());
                setOutput("error_message", "invalid_pose");
                return false;
            }

            // Transform
            // make sure any input pose is transformed to base_frame
            geometry_msgs::msg::PoseStamped base_pose;
            try
            {
                // tf2_ros::Buffer::transform takes the message by const reference.
                base_pose = tf_buffer->transform(*input_pose, base_frame);
            }
            catch (const tf2::TransformException &ex)
            {
                const std::string msg = std::string("[SpawnWallService]: TF transform failed: ") + ex.what();
                RCLCPP_ERROR(logger(), "[%s] %s", name().c_str(), msg.c_str());
                setOutput("error_message", msg);
                return false;
            }

            // Build request in base_frame
            request->pose = base_pose;
            request->pose.header.frame_id = base_frame;
            request->width = width;
            request->length = length;

            return true;
        }

        BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override
        {
            if (!response->success)
            {
                setOutput("error_message", response->error_message);
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(logger(), "[%s] wall spawned successfully", name().c_str());
            setOutput("wall_id", response->wall_id);
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
        {
            setOutput("error_message", std::string(BT::toStr(error)));
            RCLCPP_ERROR(logger(), "[%s] service failed: %d", name().c_str(), error);
            return BT::NodeStatus::FAILURE;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__COMMON__ACTIONS__SPAWN_WALL_SERVICE_HPP_
