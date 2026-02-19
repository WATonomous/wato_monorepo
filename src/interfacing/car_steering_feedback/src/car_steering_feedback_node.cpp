#include "car_steering_feedback/car_steering_feedback_node.hpp"

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cmath>
#include <cstring>

static constexpr canid_t STEERING_ANGLE_CAN_ID = 0x2B0;
static constexpr double STEERING_ANGLE_SCALAR = 0.1;

namespace car_steering_feedback
{

CarSteeringFeedbackNode::CarSteeringFeedbackNode(const rclcpp::NodeOptions & options)
: Node("car_steering_feedback_node", options)
{
  this->declare_parameter<std::string>("can_interface", "can1");
  this->declare_parameter<double>("steering_conversion_factor", 15.7);

  can_interface_ = this->get_parameter("can_interface").as_string();
  steering_conversion_factor_ = this->get_parameter("steering_conversion_factor").as_double();

  pub_ = this->create_publisher<roscco_msg::msg::SteeringAngle>(
    "car_steering_feedback/steering_angle", rclcpp::QoS(1));

  // Open SocketCAN
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to open CAN socket");
    return;
  }

  // Kernel-level filter: only deliver 0x2B0 frames to this socket
  struct can_filter filter;
  filter.can_id = STEERING_ANGLE_CAN_ID;
  filter.can_mask = CAN_SFF_MASK;
  setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to find CAN interface %s", can_interface_.c_str());
    close(sock_);
    sock_ = -1;
    return;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to bind CAN socket to %s", can_interface_.c_str());
    close(sock_);
    sock_ = -1;
    return;
  }

  RCLCPP_INFO(get_logger(), "Listening for steering angle on %s (CAN ID 0x%X)",
    can_interface_.c_str(), STEERING_ANGLE_CAN_ID);

  running_.store(true);
  read_thread_ = std::thread(&CarSteeringFeedbackNode::read_loop, this);
}

CarSteeringFeedbackNode::~CarSteeringFeedbackNode()
{
  running_.store(false);
  if (sock_ >= 0) {
    shutdown(sock_, SHUT_RDWR);
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (sock_ >= 0) {
    close(sock_);
  }
}

void CarSteeringFeedbackNode::read_loop()
{
  struct can_frame frame;

  while (running_.load(std::memory_order_relaxed)) {
    ssize_t nbytes = read(sock_, &frame, sizeof(frame));
    if (nbytes < 0) {
      if (errno == EINTR) { continue; }
      break;
    }
    if (nbytes != sizeof(frame)) { continue; }

    // Parse: int16_t little-endian from bytes 0-1, scale by 0.1, negate
    int16_t raw = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
    double steering_wheel_angle_deg = -(static_cast<double>(raw) * STEERING_ANGLE_SCALAR);

    // Convert steering wheel angle to wheel angle, then to radians
    double angle_rad = (steering_wheel_angle_deg / steering_conversion_factor_) * (M_PI / 180.0);

    roscco_msg::msg::SteeringAngle msg;
    msg.angle = static_cast<float>(angle_rad);
    msg.header.stamp = this->now();
    pub_->publish(msg);
  }
}

}  // namespace car_steering_feedback

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_steering_feedback::CarSteeringFeedbackNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
