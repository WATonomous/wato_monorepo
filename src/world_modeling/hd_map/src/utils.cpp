#include "utils.hpp"

lanelet::Polygon3d utils::boundingBox3dToPolygon3d(const lanelet::BoundingBox3d& bbox){
      auto min = bbox.min();
      auto max = bbox.max();

      lanelet::Polygon3d polygon{
          lanelet::utils::getId(),
          {
            lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), min.z()),
            lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), min.z()),
            lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), min.z()),
            lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), min.z()),
            lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), max.z()),
            lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), max.z()),
            lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), max.z()),
            lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), max.z())
          }
      };

      return polygon;
}
