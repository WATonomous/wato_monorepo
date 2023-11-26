#ifndef WORLD_MODELING_PEDESTRIAN_REG_ELEM_
#define WORLD_MODELING_PEDESTRIAN_REG_ELEM_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Exceptions.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include "hd_map_router.hpp"

class PedestrianRegElem{
    public: 
    // Constructor 
    PedestrianRegElem(lanelet::Id id, lanelet::LineString3d predictedStates);

    // gets a predicted path of the pedestrian
    lanelet::ConstLineString3d getPredictedState() const;

    // there is the isTrackDead & updatePredictedStates functions but don't fully understand them/how they will transfer to this repo
    // Think this function checks if the pedestrian is gone or not
    bool isTrackDead() const;

    private:

    // initialize variabe of type time 
    ros::Time _last_seen_time;

    // Rowans code has an explicit constructor here, need to understand it more
};

#endif