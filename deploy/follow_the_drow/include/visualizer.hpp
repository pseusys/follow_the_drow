#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include "follow_the_drow/raw_data.h"
#include "follow_the_drow/detection.h"

#include "colors.hpp"


class Visualizer {
    private:
        ros::NodeHandle handle;
        ros::Subscriber rawData, algorithmicDetector;
        ros::Publisher backVisualizer, frontVisualizer;

        bool flattenOutput = false;
        Color bottomBackground, topBackground, centerMarker;
        Color algorithmicDetection;

        std::vector<geometry_msgs::Point> latestBottomScan, latestTopScan;
        std::vector<geometry_msgs::Point> algorithmicDetectorData;

        bool scanReceived = false;
        bool algorithmicReceived = false;

        void rawDataCallback(const follow_the_drow::raw_data::ConstPtr& data);
        void algorithmicDetectorCallback(const follow_the_drow::detection::ConstPtr& data);

        void addPointToMarker(visualization_msgs::Marker& marker, const geometry_msgs::Point& point, const Color color) const;
        void addPointToMarker(visualization_msgs::Marker& marker, float x, float y, float z, const Color color) const;

        const Color readColorFromParams(const std::string& parameter) const;
        visualization_msgs::Marker initMarker(const std::string& id, const std::string& topic, float scaleRadius) const;

        void update() const;

    public:
        Visualizer();
};
