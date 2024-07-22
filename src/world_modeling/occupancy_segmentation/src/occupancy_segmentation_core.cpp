#include <string>
#include <vector>

#include "occupancy_segmentation_core.hpp"

OccupancySegmentationCore::OccupancySegmentationCore() {
  init_czm();
}

void OccupancySegmentationCore::init_czm() {
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
        pcl::PointCloud<pcl::PointXYZ> patch;
        ring.emplace_back(patch);

        pcl::PointCloud<pcl::PointXYZ> regionground;
        pcl::PointCloud<pcl::PointXYZ> regionnonground;
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

void OccupancySegmentationCore::fill_czm(pcl::PointCloud<pcl::PointXYZ> &cloud_in) {
  double lmins[4] = {L_MIN, (7 * L_MIN + L_MAX) / 8, (3 * L_MIN + L_MAX) / 4, (L_MIN + L_MAX) / 2};
  double lmaxs[4] = {lmins[1], lmins[2], lmins[3], L_MAX};
  for (pcl::PointXYZ &p : cloud_in.points) {
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
        ring_idx = (int)((r - lmins[zone_idx]) / ring_size);
        // TODO: find out why the use min() in the paper, meantime use the top
        // ring_idx = std::min((int) ((r - lmins[zone_idx]) / ring_size),);
        sector_idx = (int)(theta / sector_size);
        _czm[zone_idx][ring_idx][sector_idx].points.emplace_back(p);
      }
    }
  }
}

void OccupancySegmentationCore::clear_czm_and_regionwise(){
  int i = 0;
  for (Zone zone : _czm){
    for (Ring ring : zone){
      for (pcl::PointCloud<pcl::PointXYZ> patch : ring){
        patch.clear();
        _regionwise_ground[i].clear();
        _regionwise_nonground[i].clear();
        i++;
      }
    }
  }
}

void OccupancySegmentationCore::estimate_plane(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                               PCAFeature &feat) {
  // Code taken directly from repo
  // Eigen::Matrix3f cov;
  // Eigen::Vector4f mean;
  // pcl::computeMeanAndCovarianceMatrix(cloud, cov, mean);

  // Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  // feat.singular_values_ = svd.singularValues();

  // feat.linearity_ = (feat.singular_values_(0) - feat.singular_values_(1)) / feat.singular_values_(0);
  // feat.planarity_ = (feat.singular_values_(1) - feat.singular_values_(2)) / feat.singular_values_(0);

  // // use the least singular vector as normal
  // feat.normal_ = (svd.matrixU().col(2));
  // if (feat.normal_(2) < 0) {  // z-direction of the normal vector should be positive
  //   feat.normal_ = -feat.normal_;
  // }
  // // mean ground seeds value
  // feat.mean_ = mean.head<3>();
  // // according to normal.T*[x,y,z] = -d
  // feat.d_ = -(feat.normal_.transpose() * feat.mean_)(0, 0);
  // feat.th_dist_d_ = MD - feat.d_;
}

void OccupancySegmentationCore::segment_ground(pcl::PointCloud<pcl::PointXYZ> &unfiltered_cloud, pcl::PointCloud<pcl::PointXYZ> &ground, pcl::PointCloud<pcl::PointXYZ> &nonground) {
  // clear_czm_and_regionwise();

  // //TODO error point removal

  // fill_czm(unfiltered_cloud);

  // tbb::parallel_for(tbb::blocked_range<int>(0, num_patches), 
  //                 [&](tbb::blocked_range<int> r) {
  //   for (auto patch_num = r.begin(); patch_num != r.end(); patch_num++) {
  //     Patch_Index &p_idx = _patch_indices[patch_num];
  //     pcl::PointCloud<pcl::PointXYZ> &patch = _czm[p_idx.zone_idx][p_idx.ring_idx][p_idx.sector_idx];
  //     pcl::PointCloud<pcl::PointXYZ> &region_ground = _regionwise_ground[p_idx.idx];
  //     pcl::PointCloud<pcl::PointXYZ> &region_nonground = _regionwise_nonground[p_idx.idx];
  //     PCAFeature features;

  //     region_ground.clear();
  //     region_nonground.clear();
  //     if (patch.points.size() > MIN_NUM_POINTS) {
  //       std::sort(patch.points.begin(), patch.points.end(), point_z_cmp);
  //       rgpf(patch, p_idx, features);

  //       Status status = ground_likelihood_est(features, p_idx.concentric_idx);
  //       _statuses[p_idx.idx] = status;
  //     } else {
  //       region_ground = patch;
  //       _statuses[p_idx.idx] = FEW_POINTS;
  //     }
  //   }
  // });

  // for (Patch_Index p_idx : _patch_indices){
  //   Status status = _statuses[p_idx.idx];

  //   if (status == FEW_POINTS || status == FLAT_ENOUGH || status == UPRIGHT_ENOUGH){
  //     ground += _regionwise_ground[p_idx.idx];
  //     nonground += _regionwise_nonground[p_idx.idx];
  //   } else {
  //     nonground += _regionwise_ground[p_idx.idx];
  //     nonground += _regionwise_nonground[p_idx.idx];
  //   }

  // }
}

void OccupancySegmentationCore::rgpf(pcl::PointCloud<pcl::PointXYZ> &patch, Patch_Index &p_idx, PCAFeature &feat) {
  pcl::PointCloud<pcl::PointXYZ> ground_temp;
  pcl::PointCloud<pcl::PointXYZ> &region_ground = _regionwise_ground[p_idx.idx];
  pcl::PointCloud<pcl::PointXYZ> &region_nonground = _regionwise_nonground[p_idx.idx];

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
    for (pcl::PointXYZ &p:patch.points) {
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

Status OccupancySegmentationCore::ground_likelihood_est(PCAFeature &feat, int concentric_idx){

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

void OccupancySegmentationCore::extract_initial_seeds(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                      pcl::PointCloud<pcl::PointXYZ> &seed_cloud, int zone_idx) {
  
  //adaptive seed selection for 1st zone

  size_t init_idx = 0;
  // if (zone_idx == 0){

  // 

  double sum = 0;
  int cnt = 0;
  for (size_t i = 0; i < cloud.points.size() && cnt < NUM_SEED_POINTS; i++) {
    sum += cloud.points[i].z;
    cnt++;
  }

  double mean_z = sum / cnt;
  for (size_t i = init_idx; i < cloud.points.size(); i++) {
    if (cloud.points[i].z < mean_z + TH_SEEDS) {
      seed_cloud.points.push_back(cloud.points[i]);
    }
  }
}

