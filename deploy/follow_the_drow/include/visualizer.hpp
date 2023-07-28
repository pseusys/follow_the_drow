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
        ros::Subscriber rawData, algorithmicDetector, DROWDetector;
        ros::Publisher backVisualizer, frontVisualizer;
        const std::string frontTopic, backTopic;

        bool flattenOutput = false;
        Color bottomBackground, topBackground, centerMarker;
        Color algorithmicDetection, DROWDetection;

        std::vector<geometry_msgs::Point> latestBottomScan, latestTopScan;
        std::vector<geometry_msgs::Point> algorithmicDetectorData, DROWDetectorData;

        bool scanReceived = false;
        bool algorithmicReceived = false;
        bool DROWReceived = false;

        void rawDataCallback(const follow_the_drow::raw_data::ConstPtr& data);
        void algorithmicDetectorCallback(const follow_the_drow::detection::ConstPtr& data);
        void DROWDetectorCallback(const follow_the_drow::detection::ConstPtr& data);

        void addPointToMarker(visualization_msgs::Marker& marker, const geometry_msgs::Point& point, const Color color) const;
        void addPointToMarker(visualization_msgs::Marker& marker, double x, double y, double z, const Color color) const;

        visualization_msgs::Marker initMarker(const std::string& topic, double scaleRadius) const;
        void update() const;

    public:
        Visualizer(Color bottom, Color top, Color center, Color algorithmic, Color drow, bool flatten, const std::string& front, const std::string& back);
};
