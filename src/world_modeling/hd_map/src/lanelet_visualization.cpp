#include "lanelet_visualization.hpp"

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <boost/variant.hpp>


namespace world_modeling::hd_map
{
    visualization_msgs::msg::MarkerArray laneletMapAsMarkerArray(lanelet::LaneletMapPtr map){
        lanelet::LaneletLayer& lanelets = map->laneletLayer;
        
        auto markerArray = visualization_msgs::msg::MarkerArray();

        int id = 0;
        for (auto lanelet = lanelets.begin(); lanelet != lanelets.end(); ++lanelet){
            std_msgs::msg::ColorRGBA color;
            color.g = 1;
            color.a = 1;

            auto markers = laneletAsMarkerArray(*lanelet, &id, false, true, color, color);
            for(auto marker : markers.markers){
                markerArray.markers.push_back(marker);
            }

            // Visualize reg elems

            std::vector<std::shared_ptr<TrafficLightRegElem>> trafficLightRegElems = lanelet->regulatoryElementsAs<TrafficLightRegElem>();
            std::vector<std::shared_ptr<PedestrianRegElem>> pedestrianRegElems = lanelet->regulatoryElementsAs<PedestrianRegElem>();

            auto trafficLightMarkers = trafficLightsAsMakerArray(trafficLightRegElems);
            auto pedestrianMarkers = pedestrianAsMarkerArray(pedestrianRegElems);

            for(auto marker : trafficLightMarkers.markers){
                markerArray.markers.push_back(marker);
            }

        }

        return markerArray;
    }

    visualization_msgs::msg::MarkerArray trafficLightsAsMakerArray(std::vector<std::shared_ptr<TrafficLightRegElem>> trafficLightRegElems) {
        auto markerArray = visualization_msgs::msg::MarkerArray();

        if (trafficLightRegElems.empty()) { return markerArray; }

        for (auto trafficLight = trafficLightRegElems.begin(); trafficLight != trafficLightRegElems.end(); ++ trafficLight) {

            auto light = trafficLight->get();
            auto id = light->getId();

            // TODO get Color from light->getColor()

            auto regElemColor = std_msgs::msg::ColorRGBA();

            regElemColor.r = 1;
            regElemColor.a = 1;


            lanelet::ConstPolygon3d polygon = boost::get<lanelet::ConstPolygon3d>(light->getParameters().at(lanelet::RoleName::Refers).front());

            auto marker = polygonToMarker(polygon, &id, .1, 5, regElemColor);

            markerArray.markers.push_back(marker);  

            RCLCPP_INFO(rclcpp::get_logger("lanelet_visualization"), "Visualized traffic light on hd map: ID = %i", id);

        }

        return markerArray;
    }
    
    // TODO Test Pedestrian Visualization (pedestrianAsMarkerArray)

    visualization_msgs::msg::MarkerArray pedestrianAsMarkerArray(std::vector<std::shared_ptr<PedestrianRegElem>> pedestrianRegElems) {
        auto markerArray = visualization_msgs::msg::MarkerArray();

        if (pedestrianRegElems.empty()) { return markerArray; }

        std::shared_ptr<PedestrianRegElem> pedestrianRegElem = pedestrianRegElems.front();

        for (auto pedestrianRegElem = pedestrianRegElems.begin(); pedestrianRegElem != pedestrianRegElems.end(); ++ pedestrianRegElem) {

            auto pedestrian = pedestrianRegElem->get();
            auto id = pedestrian->getId();

            // TODO get Color from pedestrian->getColor()

            auto pedElemColor = std_msgs::msg::ColorRGBA();

            pedElemColor.r = 1;
            pedElemColor.a = 1;


            lanelet::ConstPolygon3d polygon = boost::get<lanelet::ConstPolygon3d>(pedestrian->getParameters().at(lanelet::RoleName::Refers).front());

            auto marker = polygonToMarker(polygon, &id, .1, 5, pedElemColor);

            markerArray.markers.push_back(marker);  

            RCLCPP_INFO(rclcpp::get_logger("lanelet_visualization"), "Visualized pedestrian on hd map: ID = %i", id);
        }

        return markerArray;
    }

    // TODO Add and Test Traffic Light Visualization (trafficLightsAsMarkerArray)

    visualization_msgs::msg::MarkerArray laneletAsMarkerArray(lanelet::ConstLanelet lanelet, int *id, bool center, bool lanes, std_msgs::msg::ColorRGBA centerColor, std_msgs::msg::ColorRGBA laneColor, float centerThickness, float laneThickness) {
        auto leftBound = lanelet.leftBound();
        auto rightBound = lanelet.rightBound();
        auto centerLine = lanelet.centerline();
        
        auto leftMarker = lineStringAsMarker(leftBound, id, laneThickness, 4, laneColor);
        auto rightMarker = lineStringAsMarker(rightBound, id, laneThickness, 4, laneColor);
        auto centerMarker = lineStringAsMarker(centerLine, id, centerThickness, 4, centerColor);

        auto markerArray = visualization_msgs::msg::MarkerArray();
        if(lanes) {
            markerArray.markers.push_back(leftMarker);
            markerArray.markers.push_back(rightMarker);
        }
        if(center){
            markerArray.markers.push_back(centerMarker);
        }
       
        return markerArray;
    }

    visualization_msgs::msg::MarkerArray laneletAsMarkerArray(lanelet::Lanelet lanelet, int *id, bool center, bool lanes, std_msgs::msg::ColorRGBA centerColor, std_msgs::msg::ColorRGBA laneColor, float centerThickness, float laneThickness) {
        auto leftBound = lanelet.leftBound();
        auto rightBound = lanelet.rightBound();
        auto centerLine = lanelet.centerline();

        auto leftMarker = lineStringAsMarker(leftBound, id, laneThickness, 4, laneColor);
        auto rightMarker = lineStringAsMarker(rightBound, id, laneThickness, 4, laneColor);
        auto centerMarker = lineStringAsMarker(centerLine, id, centerThickness, 4, centerColor);

        auto markerArray = visualization_msgs::msg::MarkerArray();
        if(lanes) {
            markerArray.markers.push_back(leftMarker);
            markerArray.markers.push_back(rightMarker);
        }
        if(center){
            markerArray.markers.push_back(centerMarker);
        }

        return markerArray;
    }

    visualization_msgs::msg::MarkerArray laneletPathAsMarkerArray(lanelet::routing::LaneletPath laneletPath) {        
        auto markerArray = visualization_msgs::msg::MarkerArray();

        int id = 0;

        for (auto lanelet = laneletPath.begin(); lanelet != laneletPath.end(); ++lanelet){
            auto laneColor = std_msgs::msg::ColorRGBA();
            laneColor.r = 1;
            laneColor.g = 0;
            laneColor.b = 0;
            laneColor.a = 1;
            
            auto marker = lineStringAsMarker(lanelet->centerline(), &id, 1, 4, laneColor);
            
            markerArray.markers.push_back(marker);
        }

        return markerArray;
    }

    visualization_msgs::msg::MarkerArray lineStringsAsMarkerArray(lanelet::LineStringLayer& lineStrings){
        auto markerArray = visualization_msgs::msg::MarkerArray();

        int id = 0;
        for(auto line = lineStrings.begin(); line != lineStrings.end(); ++line){
            auto color = std_msgs::msg::ColorRGBA();
            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1;
            
            auto marker = lineStringAsMarker(*line, &id, .2, 4, color);

            markerArray.markers.push_back(marker);
        }

        return markerArray;
    }

    visualization_msgs::msg::Marker lineStringAsMarker(lanelet::ConstLineString3d lineString, int *id, float thickness, int type, std_msgs::msg::ColorRGBA color){
        auto marker = visualization_msgs::msg::Marker(); 
        std_msgs::msg::Header header; // empty header
        header.stamp = rclcpp::Clock().now(); // time
        header.frame_id = "map";
        
        marker.header = header;
        marker.type = type;
        marker.id = *id;

        *id += 1;
        
        marker.color = color;

        marker.scale.x = thickness;
        marker.scale.y = 0;
        marker.scale.z = 0;

        std::vector<geometry_msgs::msg::Point> rosPoints;

        for (auto aPoint = lineString.begin(); aPoint != lineString.end(); ++aPoint) {
            auto pt = geometry_msgs::msg::Point();

            pt.x = (*aPoint).x();
            pt.y = (*aPoint).y();
            pt.z = (*aPoint).z();

            rosPoints.push_back(pt);
        }

        marker.points = rosPoints;

        return marker;
    }


    visualization_msgs::msg::Marker polygonToMarker(lanelet::ConstPolygon3d polygon, uint64_t *id, float thickness, int type, std_msgs::msg::ColorRGBA color){
        auto marker = visualization_msgs::msg::Marker();
        std_msgs::msg::Header header; // empty header
        header.stamp = rclcpp::Clock().now(); // time
        header.frame_id = "map";

        // Set basic properties
        marker.header = header;
        marker.id = *id;

        marker.color = color;

        marker.type = type; // Represent the bounding box as lines
        marker.scale.x = thickness; // Line width

        // Calculate the bounding box
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double minZ = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();
        double maxZ = std::numeric_limits<double>::lowest();

        for (const auto& point : polygon) {
            minX = std::min(minX, point.x());
            minY = std::min(minY, point.y());
            minZ = std::min(minZ, point.z());
            maxX = std::max(maxX, point.x());
            maxY = std::max(maxY, point.y());
            maxZ = std::max(maxZ, point.z());
        }

        // Define the 8 corners of the bounding box
        std::vector<geometry_msgs::msg::Point> corners(8);

        corners[0].x = minX; corners[0].y = minY; corners[0].z = minZ;
        corners[1].x = minX; corners[1].y = minY; corners[1].z = maxZ;
        corners[2].x = minX; corners[2].y = maxY; corners[2].z = minZ;
        corners[3].x = minX; corners[3].y = maxY; corners[3].z = maxZ;
        corners[4].x = maxX; corners[4].y = minY; corners[4].z = minZ;
        corners[5].x = maxX; corners[5].y = minY; corners[5].z = maxZ;
        corners[6].x = maxX; corners[6].y = maxY; corners[6].z = minZ;
        corners[7].x = maxX; corners[7].y = maxY; corners[7].z = maxZ;

        // Add lines between the corners to form the bounding box
        auto addLine = [&](int i, int j) {
            marker.points.push_back(corners[i]);
            marker.points.push_back(corners[j]);
        };

        // Connect corners to form the bounding box edges
        addLine(0, 1); addLine(0, 2); addLine(1, 3); addLine(2, 3); // Bottom face
        addLine(4, 5); addLine(4, 6); addLine(5, 7); addLine(6, 7); // Top face
        addLine(0, 4); addLine(1, 5); addLine(2, 6); addLine(3, 7); // Vertical edges

        return marker;
    }
        
}
