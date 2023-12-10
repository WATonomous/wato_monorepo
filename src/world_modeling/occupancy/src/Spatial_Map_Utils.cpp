#include <string>
#include <vector>

#include "Spatial_Map_Utils.hpp"

void add_vehicle_to_map(common_msgs::msg::Obstacle vehicle, nav_msgs::msg::OccupancyGrid::SharedPtr occupancyGridPtr){
    // let him cook
    // Goal: Take in the vehicle obstacle message and the ptr to grid, add the bounding box of vehicle to the map 

    // Read data from obstacle message
    float halfWidth = vecicle.width_along_x_axis * 0.5;
    float halfHeight = vehicle.height_along_y_axis * 0.5;
    float[36] covarianceMatrix = vehicle.pose.covariance; // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    // I think if I extract the regular x & y pos it gives me the center of the bounding box
    float center_x = covarianceMatrix[0]; // I think these are the right indexs for the pose (hopefully)
    float center_y = covarianceMatrix[7];

    // Accessing the left corner occupancy grid pose
    float xPos = occupancyGridPtr->origin.position.x;
    float yPos = occupancyGridPtr->origin.position.y;
    float resolution = occupancyGridPtr->info.resolution;
    float quaternion = occupancyGridPtr->origin.orientation.w; // not sure if i need this but will keep for now

    // Calculate the bottom left and top right points of the bounding box
    double min_x = center_x - half_width;
    double min_y = center_y - half_height;
    double max_x = center_x + half_width;
    double max_y = center_y + half_height;

    // Convert obstacle points to grid indices, i think
    int min_col = static_cast<int>((min_x - xPos) / resolution);
    int min_row = static_cast<int>((min_y - yPos) / resolution);
    int max_col = static_cast<int>((max_x - xPos) / resolution);
    int max_row = static_cast<int>((max_y - yPos) / resolution);
    
    // Set occupancy values for each grid index within the bounding box 
    for (int row = min_row; row <= max_row; row++) {
        for (int col = min_col; col <= max_col; col++) {
            int linear_index = row * occupancyGrid->info.width + col;
            occupancyGrid->data[linear_index] = 100;  
        }
    }

}


