#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "radar_vis.hpp"
#include "radar_vis_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace visualization {

RadarVis::RadarVis() {}

sensor_msgs::msg::PointCloud2 RadarVis::convert_packet_to_pointcloud(
    const radar_msgs::msg::RadarPacket::SharedPtr msg) {
  std::string radar_frame = "radar_fixed";
  sensor_msgs::msg::PointCloud2 point_cloud;
  sensor_msgs::msg::PointField point_field;

  point_cloud.header = msg->header;
  point_cloud.header.frame_id = radar_frame;
  point_cloud.height = 1;
  point_cloud.width = 0;
  point_cloud.is_bigendian = false;
  point_cloud.point_step = 16;
  point_cloud.row_step = 0;
  point_cloud.is_dense = true;
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.count = 1;

  // X Coordinate - Point Field
  point_field.name = "x";
  point_field.offset = 0;
  point_cloud.fields.push_back(point_field);

  // Y Coordinate - Point Field
  point_field.name = "y";
  point_field.offset = 4;
  point_cloud.fields.push_back(point_field);

  // Z Coordinate - Point Field
  point_field.name = "z";
  point_field.offset = 8;
  point_cloud.fields.push_back(point_field);

  // Intensity - Point Field
  point_field.name = "intensity";
  point_field.offset = 12;
  point_cloud.fields.push_back(point_field);

  // Convert data to 4 bytes (little endian)
  uint8_t *tmp_ptr;
  for (uint8_t index = 0; index < msg->detections.size(); index++) {
    // Position X - Point Cloud Conversion
    tmp_ptr = reinterpret_cast<uint8_t *>(&(msg->detections[index].pos_x));
    for (int byte = 0; byte < 4; byte++) {
      point_cloud.data.push_back(tmp_ptr[byte]);
    }

    // Position Y - Point Cloud Conversion
    tmp_ptr = reinterpret_cast<uint8_t *>(&(msg->detections[index].pos_y));
    for (int byte = 0; byte < 4; byte++) {
      point_cloud.data.push_back(tmp_ptr[byte]);
    }

    // Position Z - Point Cloud Conversion
    tmp_ptr = reinterpret_cast<uint8_t *>(&(msg->detections[index].pos_z));
    for (int byte = 0; byte < 4; byte++) {
      point_cloud.data.push_back(tmp_ptr[byte]);
    }

    // Intensity (rcs0) - Point Cloud Conversion
    float intensity = (msg->detections[index].rcs0 / 2.0) + 50.0;
    tmp_ptr = reinterpret_cast<uint8_t *>(&(intensity));
    for (int byte = 0; byte < 4; byte++) {
      point_cloud.data.push_back(tmp_ptr[byte]);
    }
    point_cloud.width++;
  }

  point_cloud.row_step = point_cloud.width * point_cloud.point_step;
  return point_cloud;
}

visualization_msgs::msg::MarkerArray RadarVis::convert_packet_to_markers(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray markers;

  std::string radar_frame = "radar_fixed";
  int id = 0;

  for (const radar_msgs::msg::RadarDetection& det : msg->detections)    
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = radar_frame;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = det.pos_x;
    m.pose.position.y = det.pos_y;
    m.pose.position.z = det.pos_z;

    double angle_det = atan2(det.pos_y, det.pos_x); // in xy plane (yaw)

    // DO EULER TO QUAT STUFF
    tf2::Quaternion quat;
    quat.setRPY(0, 0, angle_det);
    quat = quat.normalize();

    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat);

    m.pose.orientation = quat_msg;

    m.scale.x= 0.1;
    m.scale.y= 0.1;
    m.scale.z = 0.1;

    m.color.g = 0.0f;
    m.color.a = 1.0;
    m.color.r = 0.0f;
    m.color.b = 1.0f;

    ++id;
  }

  return markers;
}

}  // namespace visualization
