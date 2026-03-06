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

#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_lib.h"  // NOLINT(build/include_subdir)

using namespace std;  // NOLINT(build/namespaces)

enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};

enum Surround
{
  Prev,
  Next
};

enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

bool time_list_cut_frame(PointType & x, PointType & y);

struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;

  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  velodyne_ros::Point,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))

class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Preprocess();
  ~Preprocess();

  void process(const sensor_msgs::msg::PointCloud2 & msg, PointCloudXYZI::Ptr & pcl_out);
  void process_cut_frame_pcl2(
    const sensor_msgs::msg::PointCloud2 & msg,
    deque<PointCloudXYZI::Ptr> & pcl_out,
    deque<double> & time_lidar,
    const int required_frame_num,
    int scan_count);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128];  // maximum 128 line lidar
  vector<orgtype> typess[128];  // maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;
  double blind;
  bool feature_enabled, given_offset_time;

private:
  void velodyne_handler(const sensor_msgs::msg::PointCloud2 & msg);
  void give_feature(PointCloudXYZI & pl, vector<orgtype> & types);
  int plane_judge(
    const PointCloudXYZI & pl, vector<orgtype> & types, uint i, uint & i_nex, Eigen::Vector3d & curr_direct);
  bool edge_jump_judge(const PointCloudXYZI & pl, vector<orgtype> & types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
#endif
