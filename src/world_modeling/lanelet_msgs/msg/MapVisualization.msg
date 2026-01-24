# MapVisualization.msg
# Subset of lanelet map for visualization in RViz.
#
# Published by the lanelet node when visualization is enabled.
# Contains only lanelets within a radius of the ego vehicle to
# prevent RViz from lagging on large maps.

std_msgs/Header header

# Lanelets in the visualization region
Lanelet[] lanelets
