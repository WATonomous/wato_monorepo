#ifndef OCCUPANCY_SEGMENTATION_CORE_HPP_
#define OCCUPANCY_SEGMENTATION_CORE_HPP_

#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <Eigen/Dense>

struct PointXYZIRT {
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float intensity;
  u_int16_t ring;
  float time;
  PCL_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

struct PCAFeature {
  Eigen::Vector3f principal_;
  Eigen::Vector3f normal_;
  Eigen::Vector3f singular_values_;
  Eigen::Vector3f mean_;
  float d_;
  float th_dist_d_;
  float linearity_;
  float planarity_;
};

struct Patch_Index {
  int idx;
  int zone_idx;
  int ring_idx;
  int sector_idx;
  int concentric_idx;
};

enum Status {
  TOO_TILTED,
  FLAT_ENOUGH,
  TOO_HIGH_ELEV,
  UPRIGHT_ENOUGH,
  GLOBAL_TOO_HIGH_ELEV,
  FEW_POINTS
};

template <typename PointT>
class OccupancySegmentationCore {
 public:
  typedef std::vector<pcl::PointCloud<PointT>> Ring;
  typedef std::vector<Ring> Zone;

  int NUM_ZONES;

  float L_MIN;
  float L_MAX;

  float MD;
  float MH;

  int MIN_NUM_POINTS;
  int NUM_SEED_POINTS;

  float TH_SEEDS;
  float UPRIGHTNESS_THRESH;

  int NUM_RINGS_OF_INTEREST;
  float SENSOR_HEIGHT;
  float GLOBAL_EL_THRESH;

  std::vector<int> ZONE_RINGS;
  std::vector<int> ZONE_SECTORS;
  std::vector<float> FLATNESS_THR;
  std::vector<float> ELEVATION_THR;
  std::vector<double> lmins = {L_MIN, (7 * L_MIN + L_MAX) / 8, (3 * L_MIN + L_MAX) / 4,
                               (L_MIN + L_MAX) / 2};
  std::vector<double> lmaxs = {lmins[1], lmins[2], lmins[3], L_MAX};

  int num_patches = -1;

  bool ADAPTIVE_SELECTION_EN;

  std::vector<Zone> _czm;
  std::vector<pcl::PointCloud<PointT>> _regionwise_ground;
  std::vector<pcl::PointCloud<PointT>> _regionwise_nonground;

  std::vector<Patch_Index> _patch_indices;

  pcl::PointCloud<PointT> _ground;
  pcl::PointCloud<PointT> _non_ground;

  std::vector<Status> _statuses;

  OccupancySegmentationCore();
  OccupancySegmentationCore(int num_zones, float l_min, float l_max, float md, float mh,
                            int min_num_points, int num_seed_points, float th_seeds,
                            float uprightness_thresh, int num_rings_of_interest,
                            float sensor_height, float global_el_thresh,
                            std::vector<long int, std::allocator<long int>> &zone_rings,
                            std::vector<long int, std::allocator<long int>> &zone_sectors,
                            std::vector<double> &flatness_thr, std::vector<double> &elevation_thr,
                            bool adaptive_selection_en);

  void segment_ground(pcl::PointCloud<PointT> &unfiltered_cloud, pcl::PointCloud<PointT> &ground,
                      pcl::PointCloud<PointT> &nonground);

 private:
  void init_czm();

  void fill_czm(pcl::PointCloud<PointT> &cloud_in);

  void clear_czm_and_regionwise();

  void estimate_plane(pcl::PointCloud<PointT> &cloud, PCAFeature &feat);

  void rgpf(pcl::PointCloud<PointT> &patch, Patch_Index &p_idx, PCAFeature &feat);

  void extract_initial_seeds(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &seed_cloud,
                             int zone_idx);

  Status ground_likelihood_est(PCAFeature &feat, int concentric_idx);

  static bool point_z_cmp(PointT a, PointT b) { return a.z < b.z; };
};

#endif
