#include <algorithm>
#include <iostream>

#include "std_msgs/ColorRGBA.h"

#include "main.hpp"
#include "visualizer.hpp"


void Visualizer::raw_data_callback(const follow_the_drow::raw_data::ConstPtr& data) {
    // TODO: cast to cartesian!
    latest_bottom_scan = data->bottom_lidar;
    latest_top_scan = data->top_lidar;
    scan_received = true;
}

void Visualizer::algorithmic_detector_callback(const follow_the_drow::detection::ConstPtr& data) {
    algorithmic_detector_data = data->detection;
    algorithmic_received = true;
}

void Visualizer::add_point_to_marker(visualization_msgs::Marker& marker, const geometry_msgs::Point& point, const std_msgs::ColorRGBA& color) const {
    // TODO: read flatten value from params + colors
    geometry_msgs::Point copy = point;
    if (true) copy.z = 0;
    marker.points.push_back(copy);
    marker.colors.push_back(color);
}

void Visualizer::add_point_to_marker(visualization_msgs::Marker& marker, const geometry_msgs::Point& point, float r, float g, float b) const {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    add_point_to_marker(marker, point, color);
}

void Visualizer::add_point_to_marker(visualization_msgs::Marker& marker, float x, float y, float z, float r, float g, float b) const {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    add_point_to_marker(marker, point, r, g, b);
}

void Visualizer::update() const {
    if (!scan_received) return;
    visualization_msgs::Marker back_marker;

    back_marker.header.frame_id = general_frame;
    back_marker.header.stamp = ros::Time::now();
    back_marker.ns = back_visualization_topic;
    back_marker.id = 0;
    back_marker.type = visualization_msgs::Marker::POINTS;
    back_marker.action = visualization_msgs::Marker::ADD;

    back_marker.pose.orientation.w = 1;
    back_marker.scale.x = 0.05;
    back_marker.scale.y = 0.05;
    back_marker.color.a = 1.0;

    int laser_size = std::min(latest_bottom_scan.size(), latest_top_scan.size());
    for (int loop = 0; loop < laser_size; loop++) {
        add_point_to_marker(back_marker, latest_bottom_scan[loop], 0, 0, 1);
        add_point_to_marker(back_marker, latest_top_scan[loop], 1, 0, 0);
    }

    back_visualizer.publish(back_marker);

    if (!algorithmic_received) return;
    visualization_msgs::Marker front_marker;

    front_marker.header.frame_id = general_frame;
    front_marker.header.stamp = ros::Time::now();
    front_marker.ns = back_visualization_topic;
    front_marker.id = 0;
    front_marker.type = visualization_msgs::Marker::POINTS;
    front_marker.action = visualization_msgs::Marker::ADD;

    front_marker.pose.orientation.w = 1;
    front_marker.scale.x = 0.15;
    front_marker.scale.y = 0.15;
    front_marker.color.a = 1.0;

    for (int loop = 0; loop < algorithmic_detector_data.size(); loop++) {
        add_point_to_marker(front_marker, algorithmic_detector_data[loop], 0, 1, 0);
    }

    add_point_to_marker(front_marker, 0, 0, 0, 1, 0, 1);
    front_visualizer.publish(front_marker);
}

Visualizer::Visualizer() {
    back_visualizer = handle.advertise<visualization_msgs::Marker>(back_visualization_topic, 1);
    front_visualizer = handle.advertise<visualization_msgs::Marker>(front_visualization_topic, 1);
    raw_data = handle.subscribe(raw_data_topic, 1, &Visualizer::raw_data_callback, this);
    algorithmic_detector = handle.subscribe(algorithmic_detector_topic, 1, &Visualizer::algorithmic_detector_callback, this);

    ros::Rate rate(HEARTBEAT_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        update();
        rate.sleep();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, visualizer);
    Visualizer bsObject;
    return 0;
}
