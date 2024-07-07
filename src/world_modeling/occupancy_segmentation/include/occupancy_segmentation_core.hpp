#ifndef OCCUPANCY_SEGMENTATION_CORE_HPP_
#define OCCUPANCY_SEGMENTATION_CORE_HPP_

#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <Eigen/Dense>


#define NUM_ZONES 4
#define L_MIN 2.7
#define L_MAX 80.0

#define N_SEED 20
#define Z_SEED 0.5
#define MD 0.3
#define MH -1.1

#define MIN_NUM_POINTS 10
#define NUM_SEED_POINTS 20
#define TH_SEEDS 0.5
#define UPRIGHTNESS_THRESH 45


struct PCAFeature {
    Eigen::Vector3f principal_;
    Eigen::Vector3f normal_;
    Eigen::Vector3f singular_values_;
    Eigen::Vector3f mean_;
    float    d_;
    float    th_dist_d_;
    float    linearity_;
    float    planarity_;
};

struct Patch_Index {
  int idx;
  int zone_idx;
  int ring_idx;
  int sector_idx;
  int concentric_idx;
}

class OccupancySegmentationCore {
  public:
    typedef std::vector<pcl::PointCloud<pcl::PointXYZ>> Ring;
    typedef std::vector<Ring> Zone;
    // Size of buffer before processed messages are published.
    static constexpr int BUFFER_CAPACITY = 10;

    const int ZONE_RINGS [NUM_ZONES] = {2,4,4,4};
    const int ZONE_SECTORS [NUM_ZONES] = {16,32,54,32};

    int num_patches = -1;

    std::vector<Zone> _czm;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> _regionwise_ground;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> _regionwise_nonground;

    std::vector<Patch_Idx> _patch_indices;

    pcl::PointCloud<pcl::PointXYZ> _ground;
    pcl::Pointcloud<pcl::PointXYZ> _non_ground;


    OccupancySegmentationCore();
  
  private:

    void init_czm();

    void fill_czm(pcl::PointCloud<pcl::PointXYZ> &cloud_in);

    void estimate_plane(pcl::PointCloud<pcl::PointXYZ> &cloud, PCAFeature &feat);
     
    void rgpf(pcl::PointCloud<pcl::PointXYZ> &patch, Patch_Index &p_idx, PCAFeature &feat);
    
    void extract_initial_seeds(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &seed_cloud, int zone_idx);
    
    bool ground_likelihood_est(PCAFeature &feat, int concentric_idx);

    void segment_ground();

    bool point_z_cmp(pcl::PointXYZ a, pcl::PointXYZ b);



};


#endif  // TRANSFORMER_CORE_HPP_
