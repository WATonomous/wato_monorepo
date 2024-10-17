#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/LaneletMap.h>

namespace utils{
  lanelet::Polygon3d boundingBox3dToPolygon3d(const lanelet::BoundingBox3d& bbox);
}
