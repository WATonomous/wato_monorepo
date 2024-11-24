#ifndef ODOMETRY_NODE_HPP_
#define ODOMETRY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/Odometry.hpp>
#include "transformer_core.hpp"
#include "odometry_core.hpp"

/*
Node to take steering wheel and velocity of vehicle to track local odom
Using the ackermanse steering model for odometry

Listen to data from CAN topics and then update nav/odometry/pose message

*/

class OdometryNode : public rclcpp::Node {
  public:
    // Configure pubsub nodes to keep last 20 messages.
    // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
    static constexpr int ADVERTISING_FREQ = 20;

    //Odometry node constructor 
    OdometryNode();

  private:
    //where the publisher,subscriber and callback go
    //Add the Odometry core 
    OdometryCore odometry_;
    
    //CURRENT ABSTRACTION (CAN MESSAGES)
    rclcpp::Subscription<CAN_msgs::msg::recievedMsgs>::SharedPtr _subscriber;


    rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr _publisher;

    void subscription_callback(const CANmsgs::msg::recievedMsgs::SharedPtr raw_CAN_Data);
  
}

#endif  // TRANSFORMER_NODE_HPP_
