#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include "vision_msgs/msg/detection3_d.hpp"



namespace utils {
lanelet::Polygon3d boundingBox3dToPolygon3d(const lanelet::BoundingBox3d& bbox);
lanelet::BoundingBox3d detection3dToLaneletBBox(const vision_msgs::msg::BoundingBox3D& detection3d_bbox);
}
