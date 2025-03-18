# World Modeling HD Map

The HD map is responsible for navigation and behavioral planning for the vehicle given a high-definition map of the roads. It implements the [Dynamic Relation Graph](https://ieeexplore.ieee.org/document/9812290) for decision making from dynamic perceptions.

## Structure

The HD map is based on a central routing node, which includes IO for a lanelet2 graph. The routing graph is initialized using an open street map (.osm) file and updated dynamically using the dynamic relation graph, which receivs perception inputs. 

The HD map projector is responsible for taking open street map files (.osm) and convert them to lanelet2 graphs. 

The HD map destination service is responsible for taking a destination address or lat/long coordinates and passing it to the HD map routing graph. It will use the routing graph to convert the address or lat/long into a desination lanelet node, which will then be used by the lanelet graph for global planning.

The HD map visualization node is responsible for taking the routing graph and visualizing the lanelet map as well as the route that is currently being followed.