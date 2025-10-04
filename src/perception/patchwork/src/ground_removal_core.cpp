#include "ground_removal_core.hpp"
#include <cstring>
#include <sensor_msgs/msg/point_field.hpp>

namespace wato::perception::patchworkpp {

static float readFloat(const uint8_t* p, bool big_endian) {
  float v;
  if (!big_endian) {
    std::memcpy(&v, p, sizeof(float));
  } else {
    uint8_t tmp[4] = {p[3], p[2], p[1], p[0]};
    std::memcpy(&v, tmp, sizeof(float));
  }
  return v;
}

static void writeFloat(uint8_t* p, float v, bool big_endian) {
  if (!big_endian) {
    std::memcpy(p, &v, sizeof(float));
  } else {
    uint8_t tmp[4];
    std::memcpy(tmp, &v, sizeof(float));
    p[0] = tmp[3];
    p[1] = tmp[2];
    p[2] = tmp[1];
    p[3] = tmp[0];
  }
}

GroundRemovalCore::GroundRemovalCore(const patchwork::Params &params)
    : patchwork_(std::make_unique<patchwork::PatchWorkpp>(params)) {}

void GroundRemovalCore::process(const Eigen::MatrixX3f &cloud) {
  patchwork_->estimateGround(cloud);
}

Eigen::MatrixX3f GroundRemovalCore::getGround() const {
  return patchwork_->getGround();
}

Eigen::MatrixX3f GroundRemovalCore::getNonground() const {
  return patchwork_->getNonground();
}

double GroundRemovalCore::getTimeTaken() const {
  return patchwork_->getTimeTaken();
}

Eigen::MatrixX3f GroundRemovalCore::pointCloud2ToEigen(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg) {
  int x_idx = -1, y_idx = -1, z_idx = -1;
  for (size_t i = 0; i < cloud_msg->fields.size(); ++i) {
    const auto &f = cloud_msg->fields[i];
    if (f.name == "x") {
      x_idx = static_cast<int>(i);
    } else if (f.name == "y") {
      y_idx = static_cast<int>(i);
    } else if (f.name == "z") {
      z_idx = static_cast<int>(i);
    }
  }

  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    throw std::runtime_error("PointCloud2 missing x, y, or z field");
  }

  const auto &fx = cloud_msg->fields[static_cast<size_t>(x_idx)];
  const auto &fy = cloud_msg->fields[static_cast<size_t>(y_idx)];
  const auto &fz = cloud_msg->fields[static_cast<size_t>(z_idx)];

  if (fx.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      fy.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      fz.datatype != sensor_msgs::msg::PointField::FLOAT32) {
    throw std::runtime_error("PointCloud2 x/y/z fields must be FLOAT32");
  }

  const uint32_t width = cloud_msg->width;
  const uint32_t height = cloud_msg->height == 0 ? 1 : cloud_msg->height;
  const uint32_t point_step = cloud_msg->point_step;
  const uint32_t row_step = cloud_msg->row_step ? cloud_msg->row_step : point_step * width;

  const size_t total_points = static_cast<size_t>(width) * static_cast<size_t>(height);
  Eigen::MatrixX3f points(static_cast<int>(total_points), 3);

  const uint8_t* base = cloud_msg->data.data();
  const bool big_endian = cloud_msg->is_bigendian;

  size_t k = 0;
  for (uint32_t r = 0; r < height; ++r) {
    const uint8_t* row_ptr = base + static_cast<size_t>(r) * row_step;
    for (uint32_t c = 0; c < width; ++c, ++k) {
      const uint8_t* p = row_ptr + static_cast<size_t>(c) * point_step;

      const float x = readFloat(p + fx.offset, big_endian);
      const float y = readFloat(p + fy.offset, big_endian);
      const float z = readFloat(p + fz.offset, big_endian);

      points(static_cast<int>(k), 0) = x;
      points(static_cast<int>(k), 1) = y;
      points(static_cast<int>(k), 2) = z;
    }
  }

  return points;
}

sensor_msgs::msg::PointCloud2 GroundRemovalCore::eigenToPointCloud2(
    const Eigen::MatrixX3f &points,
    const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header = header;
  cloud_msg.height = 1;
  cloud_msg.width = static_cast<uint32_t>(points.rows());
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = false;

  cloud_msg.fields.clear();
  cloud_msg.fields.reserve(3);

  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;

  field.name = "x";
  field.offset = 0;
  cloud_msg.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  cloud_msg.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  cloud_msg.fields.push_back(field);

  cloud_msg.point_step = 12;
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

  cloud_msg.data.resize(static_cast<size_t>(cloud_msg.row_step) * cloud_msg.height);

  uint8_t* base = cloud_msg.data.data();
  const bool big_endian = cloud_msg.is_bigendian;

  for (int i = 0; i < points.rows(); ++i) {
    uint8_t* dst = base + static_cast<size_t>(i) * cloud_msg.point_step;
    writeFloat(dst + 0, points(i, 0), big_endian);
    writeFloat(dst + 4, points(i, 1), big_endian);
    writeFloat(dst + 8, points(i, 2), big_endian);
  }

  return cloud_msg;
}

}  // namespace wato::percpetion::patchworkpp
