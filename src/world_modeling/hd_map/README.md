# World Modeling HD Map

The HD map node loads and provides high-precision map data so other modules know the exact road layout, lanes, and important landmarks as well as calculating the route to the destination.

## Info Service
Other nodes may call various services to request information from HD map. Options include:
-Nearest lanelet to a GPS point
-Nearest lanelet to a XYZ point
-Nearest lanelet to the cneter of a bounding box

## Sub-System Diagram

Currently there are 3 main components present: the manager, router, and service. The lanelet visualization file exists to visualize and see the loaded map on foxglove.

## Accessing Maps

There are maps available at "/home/bolty/ament_ws/etc/maps/osm/" while inside the hd_map node dev container. The current default map is ringroad.osm, which is a osm of ring road here at the University of Waterloo. This can be adjusted in hd_map/config/params.yaml.

## Creating new services

Define a .srv file in world_modeling_msgs/srv/
Update the srv file set in CmakeLists.txt
Implement the service and its callback functions in hd_map_service.cpp
