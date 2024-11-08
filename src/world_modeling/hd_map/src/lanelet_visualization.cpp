#include "lanelet_visualization.hpp"

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>


namespace world_modeling::hd_map
{
    visualization_msgs::msg::MarkerArray laneletMapAsMarkerArray(lanelet::LaneletMapPtr map){
        lanelet::LaneletLayer& lanelets = map->laneletLayer;
        // lanelet::RegulatoryElementLayer& reg_elems = map->regulatoryElementLayer;
        
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
        }

        // Visualize reg elems

        for (auto lanelet = lanelets.begin(); lanelet != lanelets.end(); ++lanelet){
            std::vector<lanelet::TrafficLight::Ptr> trafficLightRegElems = lanelet->regulatoryElementsAs<lanelet::TrafficLight>();
            // std::vector<lanelet::TrafficLight::Ptr> trafficLightRegElems = lanelet->regulatoryElementsAs<lanelet:>();

            auto trafficLightMarkers = trafficLightsAsMakerArray(trafficLightRegElems);

            for(auto marker : trafficLightMarkers.markers){
                markerArray.markers.push_back(marker);
            }
           
        }

        return markerArray;
    }

    visualization_msgs::msg::MarkerArray trafficLightsAsMakerArray(std::vector<lanelet::TrafficLight::Ptr> trafficLightRegElems) {
        auto markerArray = visualization_msgs::msg::MarkerArray();

         if (trafficLightRegElems.empty()) { return markerArray; }

        lanelet::TrafficLight::Ptr trafficLightRegElem = trafficLightRegElems.front();

        auto trafficLights = trafficLightRegElem->trafficLights();

        if (!trafficLights.empty()) { return markerArray; }

        for (auto light = trafficLights.begin(); light != trafficLights.end(); ++ light) {

            auto marker = visualization_msgs::msg::Marker();

            marker.header.frame_id = "map";
            marker.ns = "traffic_lights";
            marker.id = light->id();
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            lanelet::LineString3d points = light->lineString().get();
        
            if (!points.empty()) {
                auto point = points.front();
                marker.pose.position.x = point.x();
                marker.pose.position.y = point.y();
                marker.pose.position.z = point.z();
            }

            std_msgs::msg::ColorRGBA color;
            color.r = 1;
            color.a = 1;

            marker.color = color;

            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;

            markerArray.markers.push_back(marker);  


            RCLCPP_INFO(rclcpp::get_logger("lanelet_visualization"), "Traffic Light with ID %i visualized.", light->id());

        }
        
        return markerArray;
    }

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
}