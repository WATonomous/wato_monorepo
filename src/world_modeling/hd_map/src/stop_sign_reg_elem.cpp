#include "stop_sign_reg_elem.hpp"

StopSignRegElem::StopSignRegElem(){}

// TODO : retrieving the information from the StopSign Obstacle message
/*
INFO : Obstacle message format (for our reference)

label : StopSign
geometry_msgs/PoseWithCovariance Pose
this includes:
1. geometry_msgs/Pose
     this includes
     1. geometry_msgs/Point
         includes:
         float64 x, y, y (no need for z in this scenario as it would be percieved by camera)
     2. geometry_msgs/Quaternion (ignore this)  
2. float64 covariance (not relevant to stop sign)

dimensions:
float64 width_along_x_axis
float64 width_along_y_axis
float64 width_along_z_axis (ignore)

Unique ID:
uint32 object_id

assuming no z component as stop sign would be percieved by camera and have no z component

*/ 
void StopSignRegElem::retrieve_stop_sign_msg(){

}

// TODO : match the retrieved co-ordinates with the appropriate lanelet, return the lanelet found
// Utilize the HDMapRouter::get_nearest_lanelet_to_gps function
lanelet::ConstLanelet StopSignRegElem::find_nearest_lanelet(){

}