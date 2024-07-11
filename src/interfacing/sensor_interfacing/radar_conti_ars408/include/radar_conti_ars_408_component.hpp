#ifndef COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_
#define COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <can_msgs/msg/frame.hpp>
#include "radar_conti_ars408_msgs/msg/filter_state_cfg.hpp"
#include <radar_conti_ars408_msgs/srv/set_filter.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rmw/qos_profiles.h"

#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "radar_msgs/msg/radar_detection.hpp"
#include "radar_msgs/msg/radar_packet.hpp"

#include <ars_408_can_defines.h>

#include "bondcpp/bond.hpp"

#include "unique_identifier_msgs/msg/uuid.hpp"

#include <unordered_map>
#include <random>

#define CAN_MAX_DLC 8

// Enum class definition
enum class FilterType {
    NOFOBJ,
    DISTANCE,
    AZIMUTH,
    VRELONCOME,
    VRELDEPART,
    RCS,
    LIFETIME,
    SIZE,
    PROBEXISTS,
    Y,
    X,
    VYRIGHTLEFT,
    VXONCOME,
    VYLEFTRIGHT,
    VXDEPART,
    UNKNOWN // Add this to handle default case
};


constexpr char DEFAULT_NODE_NAME[] = "RADAR_CONTI_ARS408";

typedef unsigned char ubyte;
typedef unsigned short int uword;

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

namespace watonomous
{

    class radar_conti_ars408 : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        radar_conti_ars408(const rclcpp::NodeOptions &options);

        /// Transition callback for state error
        /**
         * on_error callback is being called when the lifecycle node
         * enters the "error" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &
                previous_state);

        /// Transition callback for state shutting down
        /**
         * on_shutdown callback is being called when the lifecycle node
         * enters the "shutting down" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state);

        /// Transition callback for state configuring
        /**
         * on_configure callback is being called when the lifecycle node
         * enters the "configuring" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "unconfigured".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &);

        /// Transition callback for state activating
        /**
         * on_activate callback is being called when the lifecycle node
         * enters the "activating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "active" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "active"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &);

        /// Transition callback for state deactivating
        /**
         * on_deactivate callback is being called when the lifecycle node
         * enters the "deactivating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "active".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "active"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &);

        /// Transition callback for state cleaningup
        /**
         * on_cleanup callback is being called when the lifecycle node
         * enters the "cleaningup" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "unconfigured" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &);

        void setFilterService(
            const std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Request> request,
            std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Response> response);

        template <typename T>
        void declare_parameter_with_type(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string &param_name, T value) {
            if constexpr (std::is_same_v<T, int>) {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<int>(value)));
            } else if constexpr (std::is_same_v<T, double>) {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<double>(value)));
            } else if constexpr (std::is_same_v<T, float>) {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<float>(value)));
            } else if constexpr (std::is_same_v<T, uint32_t>) {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<int>(value)));
            } else if constexpr (std::is_same_v<T, uint8_t>) {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<int>(value)));
            } else {
                static_assert(!std::is_same_v<T, T>, "Unsupported type for declare_parameter_with_type");
            }
        }

        // Function to handle the retrieval of parameters based on type T
       template <typename T>
       void get_parameter_with_type(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string &param_name, T &value) {
           if constexpr (std::is_same_v<T, int>) {
               node->get_parameter(param_name, value);
           } else if constexpr (std::is_same_v<T, double>) {
               node->get_parameter(param_name, value);
           } else if constexpr (std::is_same_v<T, float>) {
               node->get_parameter(param_name, value);
           } else if constexpr (std::is_same_v<T, uint32_t>) {
               int temp_value;
               node->get_parameter(param_name, temp_value);
               value = static_cast<uint32_t>(temp_value);
           } else if constexpr (std::is_same_v<T, uint8_t>) {
               int temp_value;
               node->get_parameter(param_name, temp_value);
               value = static_cast<uint8_t>(temp_value);
           } else {
               static_assert(!std::is_same_v<T, T>, "Unsupported type for get_parameter_with_type");
           }
       }

        template <typename T>
        void initializeFilterConfig(std::string radar_name, std::string config_name, T value, T &config);


        unique_identifier_msgs::msg::UUID generateRandomUUID();
        void generateUUIDTable();

        static constexpr double covariance[] = {
            0.005,
            0.007,
            0.010,
            0.014,
            0.020,
            0.029,
            0.041,
            0.058,
            0.082,
            0.116,
            0.165,
            0.234,
            0.332,
            0.471,
            0.669,
            0.949,
            1.346,
            1.909,
            2.709,
            3.843,
            5.451,
            7.734,
            10.971,
            15.565,
            22.081,
            31.325,
            44.439,
            63.044,
            89.437,
            126.881,
            180.000,
            200.000};

    private:
        // ##############Task2################
        // create Publisher
        rclcpp::QoS qos{10};
        std::string radar_detection_topic_name_;
        std::string radar_packet_topic_name_;
        std::string filter_config_topic_name_;

        uint16_t max_radar_id = 512;
        std::vector<unique_identifier_msgs::msg::UUID> UUID_table_;

        std::vector<rclcpp_lifecycle::LifecyclePublisher<radar_conti_ars408_msgs::msg::FilterStateCfg>::SharedPtr> filter_config_publishers_;
        rclcpp_lifecycle::LifecyclePublisher<radar_msgs::msg::RadarPacket>::SharedPtr radar_packet_publisher_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_subscriber_;
        rclcpp_lifecycle::LifecyclePublisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;

        rclcpp::Service<radar_conti_ars408_msgs::srv::SetFilter>::SharedPtr set_filter_service_;

        // create can_receive_callback
        void can_receive_callback(const can_msgs::msg::Frame::SharedPtr frame);
        // create handle_object_list
        void handle_object_list(const can_msgs::msg::Frame::SharedPtr frame);
        // create publish_object_map
        void publish_object_map(int sensor_id);
        // update filter
        bool setFilter(const int &sensor_id, const int &active, const int &valid, const int &type, const int &index, const int &min_value, const int &max_value);
        void updateFilterConfig(const can_msgs::msg::Frame::SharedPtr frame, const int &sensor_id);
        void initializeFilterConfigs();
        // create map container for object list
        std::map<int, radar_msgs::msg::RadarDetection> object_map_;
        std::vector<std::map<int, radar_msgs::msg::RadarDetection>> object_map_list_;

        // create data structures for radar object list
        radar_msgs::msg::RadarPacket object_list_;
        std::vector<radar_msgs::msg::RadarPacket> object_list_list_;

        // create data structures for radar filter config
        std::vector<radar_conti_ars408_msgs::msg::FilterStateCfg> radar_filter_configs_;
        std::vector<std::vector<bool>> radar_filter_active_;
        std::vector<std::vector<bool>> radar_filter_valid_;

        // additional variables
        int operation_mode_;
        int object_count;
        int number_of_radars_;

        /// @brief whether to overwrite radar configurations on startup
        bool overwrite_configs_;

        std::unique_ptr<bond::Bond> bond_{nullptr};

        std::vector<std::string> radar_link_names_;
        std::vector<bool> filter_config_initialized_list_;

        // ##################################
    };

} // namespace radar_conti_ars408

#endif // COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_
