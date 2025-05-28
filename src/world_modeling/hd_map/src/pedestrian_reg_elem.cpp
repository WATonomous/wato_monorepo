#include "pedestrian_reg_elem.hpp"
#include "utils.hpp"

// Static factory method to create a new instance
std::shared_ptr<PedestrianRegElem> PedestrianRegElem::make(
    const lanelet::BoundingBox3d& pedestrianBBox, const PedestrianState& state, uint64_t id) {
  auto pedestrian = utils::boundingBox3dToPolygon3d(pedestrianBBox);
  lanelet::RuleParameterMap rpm = {
      {lanelet::RoleNameString::Refers, {pedestrian}}};  // Pedestrian polygon in parameters
  lanelet::AttributeMap am;

  std::shared_ptr data =
      std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

  data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
  data->attributes[lanelet::AttributeName::Subtype] = RuleName;

  return std::shared_ptr<PedestrianRegElem>(new PedestrianRegElem(data, state, id));
}

// Constructor
PedestrianRegElem::PedestrianRegElem(const lanelet::RegulatoryElementDataPtr& data,
                                     const PedestrianState& state, uint64_t id)
    : lanelet::RegulatoryElement(data), state{state}, id{id} {}

void PedestrianRegElem::updatePedestrian(const lanelet::BoundingBox3d& pedestrianBBox,
                                         const PedestrianState& state) {
  auto pedestrian = utils::boundingBox3dToPolygon3d(pedestrianBBox);
  parameters()[lanelet::RoleName::Refers].clear();
  parameters()[lanelet::RoleName::Refers].emplace_back(pedestrian);  // Update pedestrian polygon
  this->state = state;
}

uint64_t PedestrianRegElem::getId() const { return id; }

PedestrianState PedestrianRegElem::getState() const { return state; }
