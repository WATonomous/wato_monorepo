#include "utils.hpp"

lanelet::Polygon3d utils::boundingBox3dToPolygon3d(const lanelet::BoundingBox3d& bbox) {
  auto min = bbox.min();
  auto max = bbox.max();

  lanelet::Polygon3d polygon{
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), min.z()),
       lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), min.z()),
       lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), min.z()),
       lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), min.z()),
       lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), max.z()),
       lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), max.z()),
       lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), max.z()),
       lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), max.z())}};

  return polygon;
}

lanelet::BoundingBox3d utils::detection3dToLaneletBBox(const vision_msgs::msg::BoundingBox3D& bbox) {
  return lanelet::BoundingBox3d(lanelet::BasicPoint3d(bbox.center.position.x - bbox.size.x / 2,
                                                    bbox.center.position.y - bbox.size.y / 2,
                                                    bbox.center.position.z - bbox.size.z / 2),
                              lanelet::BasicPoint3d(bbox.center.position.x + bbox.size.x / 2,
                                                    bbox.center.position.y + bbox.size.y / 2,
                                                    bbox.center.position.z + bbox.size.z / 2));
}