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

class OccupancySegmentationCore {
  public:
    typedef std::vector<pcl::PointCloud<pcl::PointXYZ>> Ring;
    typedef std::vector<Ring> Zone;
    // Size of buffer before processed messages are published.
    static constexpr int BUFFER_CAPACITY = 10;

    const int ZONE_RINGS [NUM_ZONES] = {2,4,4,4};
    const int ZONE_SECTORS [NUM_ZONES] = {16,32,54,32};


    std::vector<Zone> czm;


    OccupancySegmentationCore();
  
  private:

    void init_czm();

    void fill_czm(pcl::PointCloud<pcl::PointXYZ> &cloud_in);

    void estimate_plane(pcl::PointCloud<pcl::PointXYZ> &cloud, PCAFeature &feat);



};


#endif  // TRANSFORMER_CORE_HPP_
