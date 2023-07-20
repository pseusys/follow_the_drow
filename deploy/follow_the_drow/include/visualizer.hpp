#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include "follow_the_drow/raw_data.h"
#include "follow_the_drow/detection.h"


class Visualizer {
    private:
        ros::NodeHandle handle;
        ros::Subscriber raw_data, algorithmic_detector;
        ros::Publisher back_visualizer, front_visualizer;

        std::vector<geometry_msgs::Point> latest_bottom_scan;
        std::vector<geometry_msgs::Point> latest_top_scan;

        std::vector<geometry_msgs::Point> algorithmic_detector_data;

        bool scan_received = false;
        bool algorithmic_received = false;

        void raw_data_callback(const follow_the_drow::raw_data::ConstPtr& data);
        void algorithmic_detector_callback(const follow_the_drow::detection::ConstPtr& data);

        void add_point_to_marker(visualization_msgs::Marker& marker, const geometry_msgs::Point& point, const std_msgs::ColorRGBA& color) const;
        void add_point_to_marker(visualization_msgs::Marker& marker, const geometry_msgs::Point& point, float r, float g, float b) const;
        void add_point_to_marker(visualization_msgs::Marker& marker, float x, float y, float z, float r, float g, float b) const;

        void update() const;

    public:
        Visualizer();
};
