#include <string>
#include <vector>

#include "occupancy_segmentation_core.hpp"


OccupancySegmentationCore::OccupancySegmentationCore() {}

void OccupancySegmentationCore::init_czm(){
    std::vector<Zone> temp_czm;
    czm = temp_czm;
    for(int zone_idx = 0; zone_idx < NUM_ZONES; zone_idx++){
        Zone zone;
        for(int r = 0; r < ZONE_RINGS[zone_idx]; r++){
            Ring ring;
            for (int s = 0; s < ZONE_SECTORS[zone_idx]; s++){
                pcl::PointCloud<pcl::PointXYZ> patch;
                ring.emplace_back(patch);
            }
            zone.emplace_back(ring);
        }
        czm.emplace_back(zone);
    }
}

void OccupancySegmentationCore::fill_czm(pcl::PointCloud<pcl::PointXYZ> &cloud_in){
    double lmins[4] = {L_MIN, (7*L_MIN + L_MAX) / 8, (3*L_MIN + L_MAX) / 4, (L_MIN + L_MAX) / 2};
    double lmaxs[4] = {lmins[1], lmins[2], lmins[3], L_MAX};
    for(pcl::PointXYZ &p : cloud_in.points){
        double r = sqrt(pow(p.x, 2) + pow(p.y, 2));

        // Out of range for czm
        if (r < L_MIN || r > L_MAX){
            continue;
        }

        double theta = atan2(p.y,p.x); 
        if (theta < 0){
            theta += 2*M_PI;
        }

        double deltal = L_MAX - L_MIN;
        
        for (int zone_idx = 0; zone_idx < NUM_ZONES; zone_idx++){
            int ring_idx, sector_idx;
            if (r < lmaxs[zone_idx]){
                double ring_size = deltal / ZONE_RINGS[zone_idx];
                double sector_size = 2*M_PI / ZONE_SECTORS[zone_idx];
                ring_idx = (int) ((r - lmins[zone_idx]) / ring_size);
                //TODO: find out why the use min() in the paper, meantime use the top
                //ring_idx = std::min((int) ((r - lmins[zone_idx]) / ring_size),);
                sector_idx = (int) (theta / sector_size);
                czm[zone_idx][ring_idx][sector_idx].points.emplace_back(p);
            }
        }
    }
}

void OccupancySegmentationCore::estimate_plane(pcl::PointCloud<pcl::PointXYZ> &cloud, PCAFeature &feat){
    Eigen::Matrix3f cov;
    Eigen::Vector4f mean;
    pcl::computeMeanAndCovarianceMatrix(cloud, cov, mean);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    feat.singular_values_ = svd.singularValues();

    feat.linearity_ = ( feat.singular_values_(0) - feat.singular_values_(1) ) / feat.singular_values_(0);
    feat.planarity_ = ( feat.singular_values_(1) - feat.singular_values_(2) ) / feat.singular_values_(0);

    // use the least singular vector as normal
    feat.normal_ = (svd.matrixU().col(2));
    if (feat.normal_(2) < 0) { // z-direction of the normal vector should be positive
        feat.normal_ = -feat.normal_;
    }
    // mean ground seeds value
    feat.mean_ = mean.head<3>();
    // according to normal.T*[x,y,z] = -d
    feat.d_ = -(feat.normal_.transpose() * feat.mean_)(0, 0);
    feat.th_dist_d_ = MD - feat.d_;
}