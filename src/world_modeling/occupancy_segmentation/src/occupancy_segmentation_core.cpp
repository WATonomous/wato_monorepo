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
    for(pcl::PointXYZ &p : cloud_in.points){
        double r = sqrt(pow(p.x, 2) + pow(p.y, 2));
        double theta = atan2(p.y,p.x); // EDITED!
        if (theta < 0){
            theta += 2*M_PI;
        }
        double lmins[4] = {L_MIN, (7*L_MIN + L_MAX) / 8, (3*L_MIN + L_MAX) / 4, (L_MIN + L_MAX) / 2};
        double lmaxs[4] = {lmins[1], lmins[2], lmins[3], L_MAX};
        double deltal = L_MAX - L_MIN;
        
        for (int zone_idx = 0; zone_idx < NUM_ZONES; zone_idx++){

        }



    }

}