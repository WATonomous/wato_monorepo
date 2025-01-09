#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

#include "occupancy_core.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <iostream>

// Constructor
OccupancyCore::OccupancyCore() {}
OccupancyCore::OccupancyCore(int resolution) : CELLS_PER_METER{resolution} {}

nav_msgs::msg::OccupancyGrid OccupancyCore::remove_z_dimension(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    nav_msgs::msg::OccupancyGrid output_costmap;

    // Initialize the output point cloud
    output_costmap.header = msg->header;
    output_costmap.info.map_load_time = msg->header.stamp; // Not completely sure about what timestamp to use here
    output_costmap.info.resolution = 1.0 / CELLS_PER_METER; // meters per cell

    // These offsets should always be the same but load them anyway
    int offset_x = msg->fields[0].offset;
    int offset_y = msg->fields[1].offset;
    int offset_z = msg->fields[2].offset;
    int offset_intensity = msg->fields[3].offset;

    std::vector<int> x_coords;
    std::vector<int> y_coords;
    std::vector<float> z_coords;

    int x_low = 0;
    int y_low = 0;
    int x_high = 0;
    int y_high = 0;

    size_t num_cells = 0;

    for(uint i = 0; i < msg->row_step; i += msg->point_step){
      float x, y, z, intensity;
      std::memcpy(&x, &msg->data[i + offset_x], sizeof(float));
      std::memcpy(&y, &msg->data[i + offset_y], sizeof(float));
      std::memcpy(&z, &msg->data[i + offset_z], sizeof(float));
      std::memcpy(&intensity, &msg->data[i + offset_intensity], sizeof(float));
      // the x and y have to be integers (so round after scaling up by resolution)
      int x_rounded = std::round(CELLS_PER_METER*x);
      x_coords.push_back(x_rounded);
      int y_rounded = std::round(CELLS_PER_METER*y);
      y_coords.push_back(y_rounded);
      z_coords.push_back(z); // TODO: Filter out points that have too high z values
      // TODO: Transform intensities into 0-100 range integers
      if (x_rounded >= 0){
        x_high = std::max(x_high, x_rounded);
      }
      else{
        x_low = std::min(x_low, x_rounded);
      }
      if (y_rounded >= 0){
        y_high = std::max(y_high, y_rounded);
      }
      else{
        y_low = std::min(y_low, y_rounded);
      }
      num_cells++;
    }

    // We can now get the height and width of our costmap
    int width = 1 + x_high - x_low; 
    output_costmap.info.width = width;
    int height = 1 + y_high - y_low;;
    output_costmap.info.height = height;
    int total_cells = width*height;

    output_costmap.info.origin.position.set__x(-width / (2.0 * CELLS_PER_METER));
    output_costmap.info.origin.position.set__y(-height / (2.0 * CELLS_PER_METER));

    // Initialize data as all -1 first
    std::vector<int8_t> data;
    for(int i = 0; i < total_cells; i++){
      data.push_back(-1);
    } 

    // Replace with data from point cloud
    for(size_t i = 0; i < num_cells; i++){
      // Shift x and y so the center is at width/2 and height/2
      int shifted_x = x_coords[i] + width/2;
      int shifted_y = y_coords[i] + height/2;
      int index = std::min(total_cells - 1, shifted_y*width + shifted_x);
      data[index] = 0x32; 
    }

    output_costmap.data = std::move(data);

    return output_costmap;
}
