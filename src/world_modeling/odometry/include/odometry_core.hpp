#ifndef ODOMETRY_CORE_HPP_
#define ODOMETRY_CORE_HPP_

#include <vector>

#include <nav_msgs/Odometry.hpp>


//Implementation for the internal logic for the Odometry ROS2 node.
class OdometryCore {
 public:

 public:
  /**
   * Odometry constructor.
   */
  OdometryCore();

  /**
   * update the current position
   *
   * @param msg The input CAN msg
   * @returns the processed point cloud
   */
  nav_msgs::Odometry updateOdom(CAN_msgs::msg::recievedMsgs::SharedPtr msg);

 private:

};

#endif  // ODOMETRY_CORE_HPP_
