#include "lanelet_visualization.hpp"


namespace world_modeling::hd_map
{
    visualization_msgs::msg::MarkerArray laneletMapAsMarkerArray(lanelet::LaneletMapPtr map){
        lanelet::LaneletLayer& lanelets = map->laneletLayer;
        
        auto markerArray = visualization_msgs::msg::MarkerArray();

        int id = 0;
        for (auto lanelet = lanelets .begin(); lanelet != lanelets.end(); ++lanelet){
            auto markers = laneletAsMarkerArray(*lanelet, &id);
            for(auto marker : markers.markers){
                markerArray.markers.push_back(marker);
            }
        }

        return markerArray;
    }

    visualization_msgs::msg::MarkerArray laneletAsMarkerArray(lanelet::Lanelet lanelet, int *id) {
        auto leftBound = lanelet.leftBound();
        auto rightBound = lanelet.rightBound();
        auto centerLine = lanelet.centerline();
        
        auto laneColor = std_msgs::msg::ColorRGBA();
        laneColor.r = 0;
        laneColor.g = 1;
        laneColor.b = 0;
        laneColor.a = 1;
        
        auto leftMarker = lineStringAsMarker(leftBound, id, .2, 4, laneColor);
        auto rightMarker = lineStringAsMarker(rightBound, id, .2, 4, laneColor);

        auto markerArray = visualization_msgs::msg::MarkerArray();
        markerArray.markers.push_back(leftMarker);
        markerArray.markers.push_back(rightMarker);

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