# Way.msg
# A linestring primitive from the lanelet2 map.
#
# Represents any ordered sequence of points with a unique map ID:
# lane boundaries, stop lines, traffic light outlines, sign geometries, etc.

int64 id
geometry_msgs/Point[] points
