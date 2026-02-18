#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <limits>
#include "helpers.hpp"

namespace critic{

class MppiCritic{

// implement progress tracking for traj
// 

public:
    MppiCritic() = default;

    double evaluate(const std::vector<double>& state,
                            const std::vector<double>& action,
                            const std::vector<double>& prev_action);
    void set_trajectory(const std::vector<struct State>& traj);
    double terminal_cost(double x, double y, double yaw, double v);
    
    // set occupancy grid data, called from ROS node
    void set_occupancy_grid(const std::vector<int8_t>& data, 
                            unsigned int width, unsigned int height,
                            double resolution, double origin_x, double origin_y);

    struct Params{
        // -- Progress Cost -- 
        // regular step cost 
        double w_progress = 1.0;

        // terminal step cost
        double w_terminal_progress = 10.0;

        // step deviation cost
        double w_deviation = 1.0;

        // terminal deviation cost
        double w_terminal_deviation = 10.0;

        // heading error weights:
        double w_heading = 2.0;
        double w_terminal_heading = 10.0;

        // -- Rate Costs --
        // smoothen acceleration changes
        double w_jerk = 1.0;

        // smoothen steering rate changes
        double w_steering_rate = 1.0;

        // -- Obstacle Cost --
        // obstacle avoidance weight
        double w_obstacle = 100.0;
        
        // vehicle radius for footprint checking (meters)
        double vehicle_radius = 1.5;
        
        // distance-based cost parameters
        double obstacle_inflation_radius = 2.0;  // radius around obstacles with cost (meters)
        double cost_at_obstacle = 1.0;           // cost when distance = 0
        double cost_at_inflation = 0.1;          // cost at inflation radius
    };
private:
    Params params_;

    std::vector<struct State> desired_trajectory_;

    // occupancy grid data
    std::vector<int8_t> occupancy_grid_data_;
    unsigned int grid_width_ = 0;
    unsigned int grid_height_ = 0;
    double grid_resolution_ = 0.0;
    double grid_origin_x_ = 0.0;
    double grid_origin_y_ = 0.0;
    bool has_occupancy_grid_ = false;
    
    // distance field (Euclidean Distance Transform)
    // stores distance to nearest obstacle in meters for each cell
    std::vector<double> distance_field_;
    bool has_distance_field_ = false;

    // helpers
    int find_nearest_ref_index(const State &s) const;
    double lateral_error_and_tangent(int index, const State &s, double &tangent_x, double &tangent_y) const;
    
    // obstacle cost helper
    double compute_obstacle_cost(double x, double y) const;
    
    // distance field computation
    void compute_distance_field();
    
    // get distance to nearest obstacle at world coordinates (x, y)
    double get_distance_to_obstacle(double x, double y) const;
    
    // compute cost based on distance to obstacle
    double compute_distance_based_cost(double distance) const;
    
};


}