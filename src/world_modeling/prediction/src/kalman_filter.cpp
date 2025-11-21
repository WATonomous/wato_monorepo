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

#include "prediction/kalman_filter.hpp"  // kept for compatibility; node uses internal impl by default

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <Eigen/Dense>

#include <unordered_map>
#include <string>
#include <mutex>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <cctype>

namespace fs = std::filesystem;
using std::chrono::system_clock;

using Eigen::MatrixXd;
using Eigen::VectorXd;

//
// Internal KalmanCV7 implementation (state: [x,y,z, vx,vy,vz, yaw])
// (Kept inside node file so node is self-contained.)
//
struct KalmanCV7 {
  int n = 7;
  int m = 4; // measurement: x,y,z,yaw
  VectorXd x;     // (7)
  MatrixXd P;     // (7x7)
  MatrixXd F;     // (7x7)
  MatrixXd H;     // (4x7)
  MatrixXd Q;     // (7x7)
  MatrixXd R;     // (4x4)
  double dt;

  KalmanCV7(double dt_ = 0.05,
            double q_pos = 0.1,
            double q_vel = 1.0,
            double q_yaw = 0.01,
            double r_pos = 0.5,
            double r_yaw = 0.1)
  {
    dt = dt_;
    x = VectorXd::Zero(n);
    P = MatrixXd::Identity(n, n) * 1.0;
    set_dt(dt);
    H = MatrixXd::Zero(m, n);
    H(0,0) = 1.0; H(1,1) = 1.0; H(2,2) = 1.0; H(3,6) = 1.0; // measure x,y,z,yaw
    Q = MatrixXd::Zero(n,n);
    Q(0,0) = q_pos; Q(1,1) = q_pos; Q(2,2) = q_pos;
    Q(3,3) = q_vel; Q(4,4) = q_vel; Q(5,5) = q_vel;
    Q(6,6) = q_yaw;
    R = MatrixXd::Identity(m,m);
    R(0,0) = r_pos; R(1,1) = r_pos; R(2,2) = r_pos; R(3,3) = r_yaw;
  }

  void set_dt(double newdt) {
    dt = newdt;
    F = MatrixXd::Identity(n,n);
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt; // pos += vel*dt
    // yaw kept constant (no yaw rate in F)
  }

  void init_from_measurement(const VectorXd &meas) {
    // meas = [x,y,z,yaw]
    x.setZero();
    x(0) = meas(0); x(1) = meas(1); x(2) = meas(2);
    x(3) = 0.0; x(4) = 0.0; x(5) = 0.0;
    x(6) = meas(3);
    P = MatrixXd::Identity(n,n) * 1.0;
  }

  void predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(const VectorXd &z) {
    VectorXd y = z - H * x;
    // normalize yaw residual into [-pi, pi]
    y(3) = std::atan2(std::sin(y(3)), std::cos(y(3)));
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    MatrixXd I = MatrixXd::Identity(n,n);
    P = (I - K * H) * P;
  }

  std::vector<std::pair<VectorXd,MatrixXd>> rollout_predictions(int steps) {
    std::vector<std::pair<VectorXd,MatrixXd>> out;
    VectorXd x_saved = x;
    MatrixXd P_saved = P;
    for (int i=0;i<steps;i++){
      predict();
      out.emplace_back(x, P);
    }
    x = x_saved;
    P = P_saved;
    return out;
  }
};

struct TrackState {
  KalmanCV7 kf;
  double last_stamp = 0.0;
  double l = 0.0, w = 0.0, h = 0.0;
  bool size_known = false;

  TrackState(double dt = 0.05) : kf(dt) {}
};

class KalmanFilterNode : public rclcpp::Node {
public:
  KalmanFilterNode()
  : Node("kalman_filter_node")
  {
    RCLCPP_INFO(this->get_logger(), "Kalman Filter Node started");

    // parameters (tunable)
    this->declare_parameter<std::string>("input_topic", "/perception/tracks_markers");
    this->declare_parameter<std::string>("output_topic", "/prediction/boxes_markers");
    this->declare_parameter<double>("pred_horizon", 3.0);
    this->declare_parameter<double>("pred_dt", 0.1);
    this->declare_parameter<double>("default_dt", 0.05);
    this->declare_parameter<double>("q_pos", 0.05);
    this->declare_parameter<double>("q_vel", 0.5);
    this->declare_parameter<double>("q_yaw", 0.01);
    this->declare_parameter<double>("r_pos", 0.2);
    this->declare_parameter<double>("r_yaw", 0.1);
    this->declare_parameter<std::string>("csv_out_dir", "/tmp/kf_preds");
    this->declare_parameter<int>("publish_rate_hz", 10);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    pred_horizon_ = this->get_parameter("pred_horizon").as_double();
    pred_dt_ = this->get_parameter("pred_dt").as_double();
    default_dt_ = this->get_parameter("default_dt").as_double();
    q_pos_ = this->get_parameter("q_pos").as_double();
    q_vel_ = this->get_parameter("q_vel").as_double();
    q_yaw_ = this->get_parameter("q_yaw").as_double();
    r_pos_ = this->get_parameter("r_pos").as_double();
    r_yaw_ = this->get_parameter("r_yaw").as_double();
    csv_out_dir_ = this->get_parameter("csv_out_dir").as_string();
    int publish_rate = this->get_parameter("publish_rate_hz").as_int();

    // ensure csv dir exists
    try { fs::create_directories(csv_out_dir_); } catch (...) {}

    // subscriber & publisher
    using std::placeholders::_1;
    sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      input_topic_, 10, std::bind(&KalmanFilterNode::on_detection_markers, this, _1));
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_topic_, 10);

    // timer for periodic publishing of predictions
    auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1, publish_rate)));
    timer_ = this->create_wall_timer(period_ms, std::bind(&KalmanFilterNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Subscribing '%s', publishing '%s', pred_horizon=%.2f s, pred_dt=%.3f s",
                input_topic_.c_str(), output_topic_.c_str(), pred_horizon_, pred_dt_);
  }

private:
  // incoming detection callback: update/create tracks
  void on_detection_markers(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);

    // iterate markers and use per-marker timestamp when available (fallback to node time)
    for (const auto &mk : msg->markers) {
      double now;
      if (mk.header.stamp.sec != 0 || mk.header.stamp.nanosec != 0) {
        now = static_cast<double>(mk.header.stamp.sec) + mk.header.stamp.nanosec * 1e-9;
      } else {
        now = this->now().seconds();
      }

      // make instance id unique (use ns+id) to avoid collisions when ns is reused
      std::string inst_id;
      if (mk.ns.empty()) inst_id = std::to_string(mk.id);
      else inst_id = mk.ns + "_" + std::to_string(mk.id);

      double meas_x = mk.pose.position.x;
      double meas_y = mk.pose.position.y;
      double meas_z = mk.pose.position.z;
      double meas_yaw = quat_to_yaw(mk.pose.orientation);

      double length = mk.scale.x;
      double width  = mk.scale.y;
      double height = mk.scale.z;

      auto it = tracks_.find(inst_id);
      if (it == tracks_.end()) {
        // create
        TrackState ts(default_dt_);
        ts.kf = KalmanCV7(default_dt_, q_pos_, q_vel_, q_yaw_, r_pos_, r_yaw_);
        VectorXd Z(4); Z << meas_x, meas_y, meas_z, meas_yaw;
        ts.kf.init_from_measurement(Z);
        ts.last_stamp = now;
        if (length > 0 && width > 0 && height > 0) {
          ts.l = length; ts.w = width; ts.h = height; ts.size_known = true;
        }
        tracks_.emplace(inst_id, std::move(ts));
      } else {
        // update existing
        TrackState &ts = it->second;
        double dt = now - ts.last_stamp;
        if (dt <= 0) dt = ts.kf.dt;
        ts.kf.set_dt(dt);
        ts.kf.predict();
        VectorXd Z(4); Z << meas_x, meas_y, meas_z, meas_yaw;
        ts.kf.update(Z);
        ts.last_stamp = now;
        if (length > 0 && width > 0 && height > 0) {
          ts.l = length; ts.w = width; ts.h = height; ts.size_known = true;
        }
      }
    }
  }

  // periodic publisher: rollout and publish predicted boxes for every active track
  void timer_callback() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (tracks_.empty()) {
      // nothing to publish
      return;
    }

    visualization_msgs::msg::MarkerArray out_markers;

    for (auto &kv : tracks_) {
      const std::string &inst_id = kv.first;
      TrackState &ts = kv.second;

      int pred_steps = std::max(1, int(std::round(pred_horizon_ / pred_dt_)));
      ts.kf.set_dt(pred_dt_);
      auto preds = ts.kf.rollout_predictions(pred_steps);

      // optional: write CSV per-instance
      write_predictions_to_csv(inst_id, preds, ts, pred_dt_);

      // create marker for each predicted step
      for (int s = 0; s < static_cast<int>(preds.size()); ++s) {
        const VectorXd &xpred = preds[s].first;
        double px = xpred(0), py = xpred(1), pz = xpred(2), pyaw = xpred(6);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = inst_id + "_pred";
        marker.id = s;  // step index
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = px;
        marker.pose.position.y = py;
        marker.pose.position.z = pz;
        marker.pose.orientation = yaw_to_quat(pyaw);
        marker.scale.x = ts.size_known ? ts.l : 4.0;
        marker.scale.y = ts.size_known ? ts.w : 2.0;
        marker.scale.z = ts.size_known ? ts.h : 1.6;
        double alpha = std::max(0.05, 1.0 - double(s) / double(preds.size() + 1));
        marker.color.r = 0.0f; marker.color.g = 0.8f; marker.color.b = 0.1f;
        marker.color.a = static_cast<float>(alpha);
        marker.lifetime = rclcpp::Duration::from_seconds(pred_dt_ * 1.5);
        out_markers.markers.push_back(marker);
      }
    }

    // publish all predicted markers
    pub_->publish(out_markers);
  }

  void write_predictions_to_csv(const std::string &inst_id,
                                const std::vector<std::pair<VectorXd,MatrixXd>> &preds,
                                const TrackState &ts, double dt)
  {
    // CSV: timestamp_ms,x,y,z,length,width,height,yaw
    std::string safe_id = inst_id;
    for (auto &c : safe_id) if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
    std::string fname = csv_out_dir_ + "/" + safe_id + "_preds.csv";
    std::ofstream f;
    bool exists = fs::exists(fname);
    f.open(fname, std::ios::app);
    if (!exists) {
      f << "timestamp_ms,x,y,z,length,width,height,yaw\n";
    }
    // base time in ms
    auto now = system_clock::now();
    long long base_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    for (int i = 0; i < static_cast<int>(preds.size()); ++i) {
      long long t_ms = base_ms + static_cast<long long>(std::round(1000.0 * dt * (i + 1)));
      const VectorXd &xv = preds[i].first;
      double x = xv(0), y = xv(1), z = xv(2), yaw = xv(6);
      double l = ts.size_known ? ts.l : 4.0;
      double w = ts.size_known ? ts.w : 2.0;
      double h = ts.size_known ? ts.h : 1.6;
      f << t_ms << "," << std::fixed << std::setprecision(4)
        << x << "," << y << "," << z << "," << l << "," << w << "," << h << "," << yaw << "\n";
    }
    f.close();
  }

  static double quat_to_yaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0; q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

private:
  // params & topics
  std::string input_topic_, output_topic_;
  double pred_horizon_, pred_dt_, default_dt_;
  double q_pos_, q_vel_, q_yaw_, r_pos_, r_yaw_;
  std::string csv_out_dir_;

  // ROS interfaces
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // tracks
  std::unordered_map<std::string, TrackState> tracks_;
  std::mutex mutex_;
  // KalmanFilter kalman_filter_; // left out: using per-track KalmanCV7 above
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
