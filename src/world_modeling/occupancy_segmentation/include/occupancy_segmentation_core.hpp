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

#define Z_SEED 0.5
#define MD 0.3
#define MH -1.1

#define MIN_NUM_POINTS 10
#define NUM_SEED_POINTS 20
#define TH_SEEDS 0.5
#define UPRIGHTNESS_THRESH 0.5

#define NUM_RINGS_OF_INTEREST 4
#define SENSOR_HEIGHT 1.7
#define GLOBAL_EL_THRESH 0



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
};

enum Status {TOO_TILTED, FLAT_ENOUGH, TOO_HIGH_ELEV, UPRIGHT_ENOUGH, GLOBAL_TOO_HIGH_ELEV, FEW_POINTS};

template<typename PointT>
class OccupancySegmentationCore {
  public:
    typedef std::vector<pcl::PointCloud<PointT>> Ring;
    typedef std::vector<Ring> Zone;
    // Size of buffer before processed messages are published.
    static constexpr int BUFFER_CAPACITY = 10;

    const int ZONE_RINGS [NUM_ZONES] = {2,4,4,4};
    const int ZONE_SECTORS [NUM_ZONES] = {16,32,54,32};
    const float FLATNESS_THR [NUM_ZONES] = {0.0005, 0.000725, 0.001, 0.001};
    const float ELEVATION_THR [NUM_ZONES] = {0.523, 0.746, 0.879, 1.125};


    int num_patches = -1;

    std::vector<Zone> _czm;
    std::vector<pcl::PointCloud<PointT>> _regionwise_ground;
    std::vector<pcl::PointCloud<PointT>> _regionwise_nonground;

    std::vector<Patch_Index> _patch_indices;

    pcl::PointCloud<PointT> _ground;
    pcl::PointCloud<PointT> _non_ground;
    
    std::vector<Status> _statuses;


    OccupancySegmentationCore();

    void segment_ground(pcl::PointCloud<PointT> &unfiltered_cloud, pcl::PointCloud<PointT> &ground, pcl::PointCloud<PointT> &nonground);
  
  private:

    void init_czm();

    void fill_czm(pcl::PointCloud<PointT> &cloud_in);

    void clear_czm_and_regionwise();

    void estimate_plane(pcl::PointCloud<PointT> &cloud, PCAFeature &feat);
     
    void rgpf(pcl::PointCloud<PointT> &patch, Patch_Index &p_idx, PCAFeature &feat);
    
    void extract_initial_seeds(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &seed_cloud, int zone_idx);
    
    Status ground_likelihood_est(PCAFeature &feat, int concentric_idx);

    static bool point_z_cmp(PointT a, PointT b) { return a.z < b.z; };



};


template <typename PointT>
OccupancySegmentationCore<PointT>::OccupancySegmentationCore() {
  init_czm();
}

template <typename PointT>
void OccupancySegmentationCore<PointT>::init_czm() {
  int concentric_count = 0;
  int patch_count = 0;

  for (int z = 0; z < NUM_ZONES; z++) {
    Zone zone;
    Patch_Index index;
    index.zone_idx = z;

    for (int r = 0; r < ZONE_RINGS[z]; r++) {
      Ring ring;
      index.ring_idx = r;
      index.concentric_idx = concentric_count;

      for (int s = 0; s < ZONE_SECTORS[z]; s++) {
        index.sector_idx = s;
        index.idx = patch_count;
        _patch_indices.push_back(index);
        pcl::PointCloud<PointT> patch;
        ring.emplace_back(patch);

        pcl::PointCloud<PointT> regionground;
        pcl::PointCloud<PointT> regionnonground;
        Status status;
        _regionwise_ground.push_back(regionground);
        _regionwise_nonground.push_back(regionnonground);
        
        _statuses.push_back(status);

        patch_count++;
      }
      zone.emplace_back(ring);
    }
    _czm.emplace_back(zone);
  }
  num_patches = patch_count;
}

template <typename PointT>
void OccupancySegmentationCore<PointT>::fill_czm(pcl::PointCloud<PointT> &cloud_in) {
  double lmins[4] = {L_MIN, (7 * L_MIN + L_MAX) / 8, (3 * L_MIN + L_MAX) / 4, (L_MIN + L_MAX) / 2};
  double lmaxs[4] = {lmins[1], lmins[2], lmins[3], L_MAX};
  for (PointT &p : cloud_in.points) {
    double r = sqrt(pow(p.x, 2) + pow(p.y, 2));

    // Out of range for czm
    if (r < L_MIN || r > L_MAX) {
      continue;
    }

    double theta = atan2(p.y, p.x);
    if (theta < 0) {
      theta += 2 * M_PI;
    }

    double deltal = L_MAX - L_MIN;

    for (int zone_idx = 0; zone_idx < NUM_ZONES; zone_idx++) {
      int ring_idx, sector_idx;
      if (r < lmaxs[zone_idx]) {
        double ring_size = deltal / ZONE_RINGS[zone_idx];
        double sector_size = 2 * M_PI / ZONE_SECTORS[zone_idx];
        //ring_idx = (int)((r - lmins[zone_idx]) / ring_size);
        // TODO: find out why the use min() in the paper, meantime use the top
        ring_idx = std::min((int) ((r - lmins[zone_idx]) / ring_size), ZONE_RINGS[zone_idx] - 1);
        sector_idx = std::min((int)(theta / sector_size), ZONE_SECTORS[zone_idx] - 1);
        // std::cout << "Ring: " << ring_idx << std::endl;
        // std::cout << "Sector: " << sector_idx << std::endl;
        _czm[zone_idx][ring_idx][sector_idx].points.emplace_back(p);
        break;
      }
    }
  }
}

template <typename PointT>
void OccupancySegmentationCore<PointT>::clear_czm_and_regionwise(){
  int i = 0;
  for (Zone &zone : _czm){
    for (Ring &ring : zone){
      for (pcl::PointCloud<PointT> &patch : ring){
        patch.clear();
        _regionwise_ground[i].clear();
        _regionwise_nonground[i].clear();
        i++;
      }
    }
  }
}

template <typename PointT>
void OccupancySegmentationCore<PointT>::estimate_plane(pcl::PointCloud<PointT> &cloud,
                                               PCAFeature &feat) {
  //Code taken directly from repo
  Eigen::Matrix3f cov;
  Eigen::Vector4f mean;
  pcl::computeMeanAndCovarianceMatrix(cloud, cov, mean);

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  feat.singular_values_ = svd.singularValues();

  feat.linearity_ = (feat.singular_values_(0) - feat.singular_values_(1)) / feat.singular_values_(0);
  feat.planarity_ = (feat.singular_values_(1) - feat.singular_values_(2)) / feat.singular_values_(0);

  // use the least singular vector as normal
  feat.normal_ = (svd.matrixU().col(2));
  if (feat.normal_(2) < 0) {  // z-direction of the normal vector should be positive
    feat.normal_ = -feat.normal_;
  }
  // mean ground seeds value
  feat.mean_ = mean.head<3>();
  // according to normal.T*[x,y,z] = -d
  feat.d_ = -(feat.normal_.transpose() * feat.mean_)(0, 0);
  feat.th_dist_d_ = MD - feat.d_;
}

template <typename PointT>
void OccupancySegmentationCore<PointT>::segment_ground(pcl::PointCloud<PointT> &unfiltered_cloud, pcl::PointCloud<PointT> &ground, pcl::PointCloud<PointT> &nonground) {
  clear_czm_and_regionwise();

  //TODO error point removal

  fill_czm(unfiltered_cloud);
  std::cout << "Finished filling czm" << std::endl;
  tbb::parallel_for(tbb::blocked_range<int>(0, num_patches), 
                  [&](tbb::blocked_range<int> r) {
    for (auto patch_num = r.begin(); patch_num != r.end(); patch_num++) {
      Patch_Index &p_idx = _patch_indices[patch_num];
      pcl::PointCloud<PointT> &patch = _czm[p_idx.zone_idx][p_idx.ring_idx][p_idx.sector_idx];
      pcl::PointCloud<PointT> &region_ground = _regionwise_ground[p_idx.idx];
      pcl::PointCloud<PointT> &region_nonground = _regionwise_nonground[p_idx.idx];
      PCAFeature features;

      region_ground.clear();
      region_nonground.clear();
      if (patch.points.size() > MIN_NUM_POINTS) {
        std::sort(patch.points.begin(), patch.points.end(), point_z_cmp);
        rgpf(patch, p_idx, features);

        Status status = ground_likelihood_est(features, p_idx.concentric_idx);
        _statuses[p_idx.idx] = status;
      } else {
        region_ground = patch;
        _statuses[p_idx.idx] = FEW_POINTS;
      }
    }
  });

  std::cout << "Finished estimation" << std::endl;

  for (Patch_Index p_idx : _patch_indices){
    Status status = _statuses[p_idx.idx];

    if (status == FEW_POINTS || status == FLAT_ENOUGH || status == UPRIGHT_ENOUGH){
      ground += _regionwise_ground[p_idx.idx];
      nonground += _regionwise_nonground[p_idx.idx];
    } else {
      nonground += _regionwise_ground[p_idx.idx];
      nonground += _regionwise_nonground[p_idx.idx];
    }

  }
}

template <typename PointT>
void OccupancySegmentationCore<PointT>::rgpf(pcl::PointCloud<PointT> &patch, Patch_Index &p_idx, PCAFeature &feat) {
  pcl::PointCloud<PointT> ground_temp;
  pcl::PointCloud<PointT> &region_ground = _regionwise_ground[p_idx.idx];
  pcl::PointCloud<PointT> &region_nonground = _regionwise_nonground[p_idx.idx];

  if (!region_ground.empty()) region_ground.clear();
  if (!region_nonground.empty()) region_nonground.clear();

  size_t N = patch.size();

  extract_initial_seeds(patch, ground_temp, p_idx.zone_idx);
  int num_iters = 3;
  for(int i = 0; i < num_iters; i++){
    estimate_plane(ground_temp, feat);
    ground_temp.clear();
    Eigen::MatrixXf points(N, 3);
    int j = 0;
    for (PointT &p:patch.points) {
        points.row(j++) = p.getVector3fMap();
    }

    Eigen::VectorXf result = points * feat.normal_;
    for (size_t r = 0; r < N; r++) {
            if (i < num_iters - 1) {
                if (result[r] < feat.th_dist_d_) {
                    ground_temp.points.emplace_back(patch.points[r]);
                }
            } else {
                if (result[r] < feat.th_dist_d_) {
                    region_ground.points.emplace_back(patch.points[r]);
                } else {
                    region_nonground.points.emplace_back(patch.points[r]);
                }
            }
        }

  }
}

template <typename PointT>
Status OccupancySegmentationCore<PointT>::ground_likelihood_est(PCAFeature &feat, int concentric_idx){

  //uprightness filter
  if(std::abs(feat.normal_(2)) < UPRIGHTNESS_THRESH){
    return TOO_TILTED;
  }

  const double z_elevation = feat.mean_(2);
  const double surface_variable = feat.singular_values_.minCoeff() / (feat.singular_values_(0) + feat.singular_values_(1) + feat.singular_values_(2));

  //elevation filter
  if (concentric_idx < NUM_RINGS_OF_INTEREST) {
    if (z_elevation > -SENSOR_HEIGHT + ELEVATION_THR[concentric_idx]) {
        if (FLATNESS_THR[concentric_idx] <= surface_variable){
          return FLAT_ENOUGH;
        } else {
          return TOO_HIGH_ELEV;
        }
    } else{
      return UPRIGHT_ENOUGH;
    }
  } else {
    if (z_elevation > GLOBAL_EL_THRESH) {
      return GLOBAL_TOO_HIGH_ELEV;
    } else {
      return UPRIGHT_ENOUGH;
    }
  }

}

template <typename PointT>
void OccupancySegmentationCore<PointT>::extract_initial_seeds(pcl::PointCloud<PointT> &cloud,
                                                      pcl::PointCloud<PointT> &seed_cloud, int zone_idx) {
  
  //adaptive seed selection for 1st zone

  size_t init_idx = 0;
  if (zone_idx == 0){
    double adaptive_seed_selection_margin = MH * SENSOR_HEIGHT;
    for (size_t i = 0; i < cloud.points.size(); i++){
      if (cloud.points[i].z < adaptive_seed_selection_margin){
        init_idx++;
      } else{
        break;
      }
    }
  }
  

  double sum = 0;
  int cnt = 0;
  for (size_t i = 0; i < cloud.points.size() && cnt < NUM_SEED_POINTS; i++) {
    sum += cloud.points[i].z;
    cnt++;
  }

  double mean_z = (cnt != 0) ? sum / cnt : 0;
  for (size_t i = init_idx; i < cloud.points.size(); i++) {
    if (cloud.points[i].z < mean_z + TH_SEEDS) {
      seed_cloud.points.push_back(cloud.points[i]);
    }
  }
}


#endif  // TRANSFORMER_CORE_HPP_
