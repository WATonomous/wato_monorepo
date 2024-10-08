#ifndef OCCUPANCY_CORE_HPP_
#define OCCUPANCY_CORE_HPP_

#include <vector>
#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"


class OccupancyCore {
 public:
  // Size of buffer before processed messages are published.
  static constexpr int BUFFER_CAPACITY = 10;

  /**
   * OccupancyCore constructor.
   */
  OccupancyCore();

  /**
   * Retrieve enqueued messages in buffer.
   *
   * @returns enqueued messages
   */
  std::vector<sensor_msgs::msg::PointCloud2> buffer_messages() const;

  /**
   * Removes all messages in buffer. Called after messages have been published.
   */
  void clear_buffer();


  /**
   * Enqueue message into an array of processed messages to "filtered" topic.
   * Ignores messages once the buffer capacity is reached.
   *
   * @param msg a processed message to be published
   * @returns whether buffer is full after adding new message
   */
  bool enqueue_message(const sensor_msgs::msg::PointCloud2& msg);

  /**
   * Removes the z-axis dimension from the given PointCloud2 message.
   *
   * @param msg The input PointCloud2 message
   * @returns the processed point cloud
   */
  sensor_msgs::msg::PointCloud2 remove_z_dimension(sensor_msgs::msg::PointCloud2::SharedPtr msg);
 
 private:
  // Buffer storing processed messages until BUFFER_CAPACITY. Clear after
  // messages are published.
  std::vector<sensor_msgs::msg::PointCloud2> buffer_;
};

#endif // OCCUPANCY_HPP
