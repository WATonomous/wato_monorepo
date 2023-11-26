#include "pedestrian_reg_elem.hpp"

// returns the predicted path of the pedestrian 
// need to update to interface with perception
lanelet::ConstLineString3d PedRegElem::getPredictedStates() const {
  return getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine)
      .front();
}

// Checks if we havent seen the ped in over 5 seconds I think
// need to update to interface with perception
bool PedRegElem::isTrackDead() const {
  return (ros::Time::now() - _last_seen_time).toSec() > 5;
}

// updates the predeicted state with the new real state 
// update with perception omegalul
void PedRegElem::updatePredictedStates(lanelet::LineString3d predicted_states) {
  parameters()[lanelet::RoleNameString::RefLine] = {predicted_states};
  _last_seen_time = ros::Time::now();
}

/* there are a couple more functions in the DRG implementation but 
I am not sure how many of them are required now so will leave them out for now */

