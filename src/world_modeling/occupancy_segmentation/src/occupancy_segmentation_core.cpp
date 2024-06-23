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