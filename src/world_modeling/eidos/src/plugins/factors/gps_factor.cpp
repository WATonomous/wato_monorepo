#include "eidos/plugins/factors/gps_factor.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/navigation/GPSFactor.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "eidos/slam_core.hpp"

namespace eidos {

// Named constants
constexpr double kGpsTimeWindow = 0.2;                 // seconds, ± window for matching GPS to state
constexpr double kElevationLockedNoise = 0.01;         // variance when elevation disabled (locks z)
constexpr double kQuatLength2Min = 0.01;               // minimum q.length2() to accept IMU orientation

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void GpsFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".gps_topic", "gps/fix");
  node_->declare_parameter(prefix + ".max_cov", 2.0);
  node_->declare_parameter(prefix + ".use_elevation", false);
  node_->declare_parameter(prefix + ".min_radius", 5.0);
  node_->declare_parameter(prefix + ".gps_cov", std::vector<double>{1.0, 1.0, 1.0});
  node_->declare_parameter(prefix + ".imu_topic", "imu/data");
  node_->declare_parameter(prefix + ".pose_cov_threshold", 25.0);
  node_->declare_parameter(prefix + ".add_factors", true);

  // ---- Read parameters ----
  std::string gps_topic, imu_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".max_cov", max_cov_);
  node_->get_parameter(prefix + ".use_elevation", use_elevation_);
  node_->get_parameter(prefix + ".min_radius", min_radius_);
  node_->get_parameter(prefix + ".gps_cov", gps_cov_);
  node_->get_parameter(prefix + ".pose_cov_threshold", pose_cov_threshold_);
  node_->get_parameter(prefix + ".add_factors", add_factors_);

  // Read frame names
  node_->get_parameter("frames.map", map_frame_);

  // ---- Create subscriptions ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsFactor::gpsCallback, this, std::placeholders::_1),
      sub_opts);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsFactor::imuCallback, this, std::placeholders::_1),
      sub_opts);

  // ---- Static TF broadcaster for utm → map (only when publish_tf is true) ----
  if (publishesTf()) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }

  // ---- Publisher for raw GPS in UTM frame ----
  utm_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      name_ + "/utm_pose", 10);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (unary GPSFactor, self-contained UTM)", name_.c_str());
}

void GpsFactor::activate() {
  active_ = true;
  utm_pub_->on_activate();

  // Check if a prior offset was loaded from a saved map
  auto& map_manager = core_->getMapManager();
  auto global = map_manager.getGlobalData("gps_factor/utm_to_map");
  if (global.has_value()) {
    auto offset = std::any_cast<std::array<double, 5>>(global.value());
    utm_to_map_offset_ = gtsam::Point3(offset[0], offset[1], offset[2]);
    int encoded = static_cast<int>(offset[3]);
    utm_zone_ = encoded / 10;
    utm_is_north_ = (encoded % 10) != 0;
    initial_yaw_ = offset[4];
    double cy = std::cos(-initial_yaw_);
    double sy = std::sin(-initial_yaw_);
    R_map_enu_ << cy, -sy, 0,
                  sy,  cy, 0,
                   0,   0, 1;
    offset_initialized_ = true;
    broadcastUtmToMap();
    RCLCPP_INFO(node_->get_logger(),
                "[%s] loaded prior offset (zone %d%c, yaw=%.3f): [%.1f, %.1f, %.1f]",
                name_.c_str(), utm_zone_, utm_is_north_ ? 'N' : 'S',
                initial_yaw_,
                utm_to_map_offset_.x(), utm_to_map_offset_.y(), utm_to_map_offset_.z());
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void GpsFactor::deactivate() {
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void GpsFactor::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.clear();
  has_last_gps_ = false;
  offset_initialized_ = false;
  utm_to_map_offset_ = gtsam::Point3(0, 0, 0);
  has_imu_orientation_.store(false, std::memory_order_relaxed);
  initial_yaw_ = 0.0;
  R_map_enu_ = Eigen::Matrix3d::Identity();
}

// ---------------------------------------------------------------------------
// isReady - always ready (latching-only plugin, never blocks SLAM startup)
// ---------------------------------------------------------------------------
bool GpsFactor::isReady() const {
  return true;
}

std::string GpsFactor::getReadyStatus() const {
  if (!gps_received_) return "ready (no NavSatFix yet, will latch when available)";
  if (!offset_initialized_) return "ready (waiting for IMU heading to initialize offset)";
  return "GPS active";
}

// ---------------------------------------------------------------------------
// latchFactors - attach unary GPSFactor to an existing state
// ---------------------------------------------------------------------------
StampedFactorResult GpsFactor::latchFactors(gtsam::Key key, double timestamp) {
  StampedFactorResult result;

  if (!active_) return result;

  std::lock_guard<std::mutex> lock(gps_lock_);

  if (gps_queue_.empty()) return result;

  // Time-match GPS fix to state timestamp (±0.2s window, pick closest)
  // Discard fixes older than the window
  while (!gps_queue_.empty() &&
         stamp2Sec(gps_queue_.front().header.stamp) < timestamp - kGpsTimeWindow) {
    gps_queue_.pop_front();
  }

  if (gps_queue_.empty()) return result;

  // If the next fix is too new, keep it for a future state
  if (stamp2Sec(gps_queue_.front().header.stamp) > timestamp + kGpsTimeWindow) {
    return result;
  }

  // Find the fix closest to the state timestamp within the window
  size_t best_idx = 0;
  double best_dt = std::abs(stamp2Sec(gps_queue_[0].header.stamp) - timestamp);
  for (size_t i = 1; i < gps_queue_.size(); ++i) {
    double t = stamp2Sec(gps_queue_[i].header.stamp);
    if (t > timestamp + kGpsTimeWindow) break;  // past the window
    double dt = std::abs(t - timestamp);
    if (dt < best_dt) {
      best_dt = dt;
      best_idx = i;
    }
  }

  // Skip GPS if SLAM pose covariance is small (optimizer is confident)
  auto pose_cov = core_->getLatestPoseCovariance();
  if (pose_cov.has_value()) {
    // ISAM2 marginal covariance is 6x6 [rot3 | pos3]: x=3,3  y=4,4
    double cov_xx = (*pose_cov)(3, 3);
    double cov_yy = (*pose_cov)(4, 4);
    if (cov_xx < pose_cov_threshold_ && cov_yy < pose_cov_threshold_) {
      gps_queue_.clear();
      return result;
    }
  }

  sensor_msgs::msg::NavSatFix this_fix = gps_queue_[best_idx];
  // Remove all fixes up to and including the one we picked
  gps_queue_.erase(gps_queue_.begin(), gps_queue_.begin() + best_idx + 1);

  // Filter by status
  if (this_fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return result;
  }

  // Sensor-reported covariance
  double sensor_cov_x = this_fix.position_covariance[0];
  double sensor_cov_y = this_fix.position_covariance[4];
  double sensor_cov_z = this_fix.position_covariance[8];

  if (sensor_cov_x > max_cov_ || sensor_cov_y > max_cov_) {
    return result;
  }

  // Convert lat/lon to UTM
  UtmCoordinate utm = latLonToUtm(
      this_fix.latitude, this_fix.longitude, this_fix.altitude);
  Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);

  // Initialize UTM→map offset on first accepted fix
  if (!offset_initialized_) {
    if (!has_imu_orientation_.load(std::memory_order_relaxed)) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[%s] waiting for IMU orientation before initializing GPS offset",
                           name_.c_str());
      return result;
    }

    utm_zone_ = utm.zone;
    utm_is_north_ = utm.is_north;

    // Capture IMU yaw and build rotation from ENU/UTM to map frame
    {
      std::lock_guard<std::mutex> imu_lock(imu_orientation_lock_);
      initial_yaw_ = latest_imu_yaw_;
    }
    double cy = std::cos(-initial_yaw_);
    double sy = std::sin(-initial_yaw_);
    R_map_enu_ << cy, -sy, 0,
                  sy,  cy, 0,
                   0,   0, 1;

    // Compute fixed offset: offset = R_map_enu * utm_pos - map_pos
    Eigen::Vector3d utm_rotated = utmToMap(utm_pos);
    auto current_pose = core_->getCurrentPose();
    Eigen::Vector3d map_pos(current_pose.translation().x(),
                            current_pose.translation().y(),
                            current_pose.translation().z());
    utm_to_map_offset_ = gtsam::Point3(
        utm_rotated.x() - map_pos.x(),
        utm_rotated.y() - map_pos.y(),
        use_elevation_ ? (utm_rotated.z() - map_pos.z()) : 0.0);

    offset_initialized_ = true;
    broadcastUtmToMap();

    // Persist offset for map save/load
    auto& map_manager = core_->getMapManager();
    double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
    std::array<double, 5> global_offset = {
        utm_to_map_offset_.x(), utm_to_map_offset_.y(), utm_to_map_offset_.z(),
        zone_encoded, initial_yaw_};
    map_manager.setGlobalData("gps_factor/utm_to_map", global_offset);

    RCLCPP_INFO(node_->get_logger(),
                "[%s] offset initialized: yaw=%.1f deg, offset=(%.1f, %.1f, %.1f)",
                name_.c_str(), initial_yaw_ * 180.0 / M_PI,
                utm_to_map_offset_.x(), utm_to_map_offset_.y(), utm_to_map_offset_.z());
  }

  // Convert UTM position to map frame: gps_map = R_map_enu * utm_pos - offset
  Eigen::Vector3d gps_rotated = utmToMap(utm_pos);
  double gps_x = gps_rotated.x() - utm_to_map_offset_.x();
  double gps_y = gps_rotated.y() - utm_to_map_offset_.y();
  double gps_z;
  if (use_elevation_) {
    gps_z = gps_rotated.z() - utm_to_map_offset_.z();
  } else {
    // LIO-SAM style: use current SLAM z (don't pull toward 0)
    gps_z = core_->getCurrentPose().translation().z();
  }

  // Distance gate in UTM space (avoids injecting too many factors)
  float utm_x = static_cast<float>(utm_pos.x());
  float utm_y = static_cast<float>(utm_pos.y());
  if (has_last_gps_) {
    float dx = utm_x - last_gps_point_.x;
    float dy = utm_y - last_gps_point_.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_radius_) {
      return result;
    }
  }

  last_gps_point_.x = utm_x;
  last_gps_point_.y = utm_y;
  last_gps_point_.z = static_cast<float>(utm_pos.z());
  has_last_gps_ = true;

  // Build noise model: max of sensor covariance and configured floor
  double noise_x = std::max(sensor_cov_x, gps_cov_[0]);
  double noise_y = std::max(sensor_cov_y, gps_cov_[1]);
  // When elevation disabled: z = current SLAM z, lock it tight (LIO-SAM uses 0.01)
  double noise_z = use_elevation_ ? std::max(sensor_cov_z, gps_cov_[2]) : kElevationLockedNoise;
  auto gps_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(3) << noise_x, noise_y, noise_z).finished());

  // Unary GPSFactor — no shared bias variable
  gtsam::Point3 gps_measurement(gps_x, gps_y, gps_z);
  if (add_factors_) {
    auto gps_factor = gtsam::make_shared<gtsam::GPSFactor>(
        key, gps_measurement, gps_noise);
    result.factors.push_back(gps_factor);
  }

  // Store map-frame position in MapManager
  core_->getMapManager().addKeyframeData(
      key, "gps_factor/position", gps_measurement);

  // Store UTM position for relocalization
  double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
  Eigen::Vector4d utm_data(utm.easting, utm.northing, utm.altitude, zone_encoded);
  core_->getMapManager().addKeyframeData(
      key, "gps_factor/utm_position", utm_data);

  gtsam::Symbol sym(key);
  RCLCPP_INFO(node_->get_logger(),
              "\033[35m[%s] latched GPSFactor at key (%c,%lu): "
              "map=(%.3f, %.3f, %.3f) cov=(%.4f, %.4f, %.4f)\033[0m",
              name_.c_str(), sym.chr(), sym.index(),
              gps_x, gps_y, gps_z,
              noise_x, noise_y, noise_z);

  return result;
}

// ---------------------------------------------------------------------------
// broadcastUtmToMap — publish static TF using the fixed offset + heading
// ---------------------------------------------------------------------------
void GpsFactor::broadcastUtmToMap() {
  if (!publishesTf() || !static_tf_broadcaster_) return;

  // offset = R_map_enu * utm_pos - map_pos
  // So: map_pos = R_map_enu * utm_pos - offset
  //
  // TF convention (parent=utm, child=map):
  // p_utm = R_enu_map * p_map + R_enu_map * offset
  Eigen::Matrix3d R_enu_map = R_map_enu_.transpose();
  Eigen::Vector3d offset_vec(utm_to_map_offset_.x(), utm_to_map_offset_.y(), utm_to_map_offset_.z());
  Eigen::Vector3d t_tf = R_enu_map * offset_vec;
  Eigen::Quaterniond q_enu_map(R_enu_map);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node_->now();
  tf.header.frame_id = utm_frame_;
  tf.child_frame_id = map_frame_;
  tf.transform.translation.x = t_tf.x();
  tf.transform.translation.y = t_tf.y();
  tf.transform.translation.z = t_tf.z();
  tf.transform.rotation.x = q_enu_map.x();
  tf.transform.rotation.y = q_enu_map.y();
  tf.transform.rotation.z = q_enu_map.z();
  tf.transform.rotation.w = q_enu_map.w();

  static_tf_broadcaster_->sendTransform(tf);
}

// ---------------------------------------------------------------------------
// IMU callback — capture latest yaw for heading alignment
// ---------------------------------------------------------------------------
void GpsFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  if (q.length2() < kQuatLength2Min) return;

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  std::lock_guard<std::mutex> lock(imu_orientation_lock_);
  latest_imu_yaw_ = yaw;
  has_imu_orientation_.store(true, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// utmToMap — rotate UTM position into map frame using initial heading
// ---------------------------------------------------------------------------
Eigen::Vector3d GpsFactor::utmToMap(const Eigen::Vector3d& utm_pos) const {
  return R_map_enu_ * utm_pos;
}

// ---------------------------------------------------------------------------
// GPS callback
// ---------------------------------------------------------------------------
void GpsFactor::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.push_back(*msg);
  if (!gps_received_) {
    gps_received_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "[%s] first NavSatFix received: lat=%.7f lon=%.7f status=%d cov=[%.3f, %.3f, %.3f]",
                name_.c_str(), msg->latitude, msg->longitude, msg->status.status,
                msg->position_covariance[0], msg->position_covariance[4],
                msg->position_covariance[8]);
  }

  // Publish raw GPS position in UTM frame
  if (active_) {
    UtmCoordinate utm = latLonToUtm(msg->latitude, msg->longitude, msg->altitude);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = utm_frame_;
    pose.pose.position.x = utm.easting;
    pose.pose.position.y = utm.northing;
    pose.pose.position.z = utm.altitude;
    {
      std::lock_guard<std::mutex> imu_lock(imu_orientation_lock_);
      tf2::Quaternion q;
      q.setRPY(0, 0, latest_imu_yaw_);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    }
    utm_pub_->publish(pose);
  }
}

// ---------------------------------------------------------------------------
// saveData / loadData — persist GPS measurements + utm_to_map config
// ---------------------------------------------------------------------------
void GpsFactor::saveData(const std::string& plugin_dir) {
  namespace fs = std::filesystem;
  auto& map_manager = core_->getMapManager();
  auto key_list = map_manager.getKeyList();

  // Save utm_to_map config
  auto global = map_manager.getGlobalData("gps_factor/utm_to_map");
  if (global.has_value()) {
    auto offset = std::any_cast<std::array<double, 5>>(global.value());
    std::ofstream ofs(plugin_dir + "/config.bin", std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(offset.data()), 5 * sizeof(double));
  }

  // Save sparse GPS measurements (only states that have GPS data)
  auto meas_dir = fs::path(plugin_dir) / "measurements";
  fs::create_directories(meas_dir);
  for (size_t i = 0; i < key_list.size(); i++) {
    auto pos_data = map_manager.getKeyframeData(key_list[i], "gps_factor/position");
    auto utm_data = map_manager.getKeyframeData(key_list[i], "gps_factor/utm_position");
    if (!pos_data.has_value()) continue;

    char filename[32];
    snprintf(filename, sizeof(filename), "%06zu.bin", i);
    std::ofstream ofs((meas_dir / filename).string(), std::ios::binary);

    auto pos = std::any_cast<gtsam::Point3>(pos_data.value());
    double vals[3] = {pos.x(), pos.y(), pos.z()};
    ofs.write(reinterpret_cast<const char*>(vals), 3 * sizeof(double));

    if (utm_data.has_value()) {
      auto utm = std::any_cast<Eigen::Vector4d>(utm_data.value());
      double uvals[4] = {utm[0], utm[1], utm[2], utm[3]};
      ofs.write(reinterpret_cast<const char*>(uvals), 4 * sizeof(double));
    }
  }
}

void GpsFactor::loadData(const std::string& plugin_dir) {
  namespace fs = std::filesystem;
  auto& map_manager = core_->getMapManager();
  auto key_list = map_manager.getKeyList();

  // Load utm_to_map config
  auto config_path = fs::path(plugin_dir) / "config.bin";
  if (fs::exists(config_path)) {
    std::ifstream ifs(config_path.string(), std::ios::binary);
    std::array<double, 5> vals;
    ifs.read(reinterpret_cast<char*>(vals.data()), 5 * sizeof(double));
    map_manager.setGlobalData("gps_factor/utm_to_map", vals);
  }

  // Load sparse GPS measurements
  auto meas_dir = fs::path(plugin_dir) / "measurements";
  if (!fs::exists(meas_dir)) return;

  for (size_t i = 0; i < key_list.size(); i++) {
    char filename[32];
    snprintf(filename, sizeof(filename), "%06zu.bin", i);
    auto path = meas_dir / filename;
    if (!fs::exists(path)) continue;

    std::ifstream ifs(path.string(), std::ios::binary);
    double vals[3];
    ifs.read(reinterpret_cast<char*>(vals), 3 * sizeof(double));
    map_manager.addKeyframeData(key_list[i], "gps_factor/position",
                                gtsam::Point3(vals[0], vals[1], vals[2]));

    double uvals[4];
    if (ifs.read(reinterpret_cast<char*>(uvals), 4 * sizeof(double))) {
      map_manager.addKeyframeData(key_list[i], "gps_factor/utm_position",
                                  Eigen::Vector4d(uvals[0], uvals[1], uvals[2], uvals[3]));
    }
  }
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsFactor, eidos::FactorPlugin)
