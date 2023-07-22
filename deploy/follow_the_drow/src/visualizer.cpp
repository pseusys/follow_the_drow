#include <algorithm>
#include <stdexcept>

#include "main.hpp"
#include "transformation.hpp"
#include "visualizer.hpp"

#define BACKGROUND_RADIUS 0.05
#define DETECTION_RADIUS 0.15


void Visualizer::rawDataCallback(const follow_the_drow::raw_data::ConstPtr& data) {
    latestBottomScan = std::vector<geometry_msgs::Point>(data->bottom_lidar.size());
    for (int i = 0; i < data->bottom_lidar.size(); i++)
        latestBottomScan[i] = polarToCartesian(data->bottom_lidar[i]);

    latestTopScan = std::vector<geometry_msgs::Point>(data->top_lidar.size());
    for (int i = 0; i < data->top_lidar.size(); i++)
        latestTopScan[i] = polarToCartesian(data->top_lidar[i]);

    scanReceived = true;
}

void Visualizer::algorithmicDetectorCallback(const follow_the_drow::detection::ConstPtr& data) {
    algorithmicDetectorData = data->detection;
    algorithmicReceived = true;
}

void Visualizer::addPointToMarker(visualization_msgs::Marker& marker, const geometry_msgs::Point& point, const Color color) const {
    geometry_msgs::Point copy = point;
    if (flattenOutput) copy.z = 0;
    marker.points.push_back(copy);
    marker.colors.push_back(getRGBAFromColor(color));
}

void Visualizer::addPointToMarker(visualization_msgs::Marker& marker, float x, float y, float z, const Color color) const {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    addPointToMarker(marker, point, color);
}

visualization_msgs::Marker Visualizer::initMarker(const std::string& id, const std::string& topic, float scaleRadius) const {
    visualization_msgs::Marker marker;

    marker.header.frame_id = id;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;
    marker.scale.x = scaleRadius;
    marker.scale.y = scaleRadius;
    marker.color.a = 1.0;

    return marker;
}

void Visualizer::update() const {
    if (scanReceived) {
        visualization_msgs::Marker back_marker = initMarker(GENERAL_FRAME, BACK_VISUALIZATION_TOPIC, BACKGROUND_RADIUS);

        for (int loop = 0; loop < latestBottomScan.size(); loop++)
            addPointToMarker(back_marker, latestBottomScan[loop], bottomBackground);
        for (int loop = 0; loop < latestTopScan.size(); loop++)
            addPointToMarker(back_marker, latestTopScan[loop], topBackground);

        backVisualizer.publish(back_marker);
    }

    visualization_msgs::Marker front_marker = initMarker(GENERAL_FRAME, FRONT_VISUALIZATION_TOPIC, DETECTION_RADIUS);;

    if (algorithmicReceived) {
        for (int loop = 0; loop < algorithmicDetectorData.size(); loop++)
            addPointToMarker(front_marker, algorithmicDetectorData[loop], algorithmicDetection);
    }

    addPointToMarker(front_marker, 0, 0, 0, centerMarker);
    frontVisualizer.publish(front_marker);
}

Visualizer::Visualizer(Color bottom, Color top, Color center, Color algorithmic, bool flatten): bottomBackground(bottom), topBackground(top), centerMarker(center), algorithmicDetection(algorithmic), flattenOutput(flatten) {
    backVisualizer = handle.advertise<visualization_msgs::Marker>(BACK_VISUALIZATION_TOPIC, 1);
    frontVisualizer = handle.advertise<visualization_msgs::Marker>(FRONT_VISUALIZATION_TOPIC, 1);
    rawData = handle.subscribe(RAW_DATA_TOPIC, 1, &Visualizer::rawDataCallback, this);
    algorithmicDetector = handle.subscribe(ALGORITHMIC_DETECTOR_TOPIC, 1, &Visualizer::algorithmicDetectorCallback, this);

    ros::Rate rate(HEARTBEAT_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        update();
        rate.sleep();
    }
}


const Color readColorFromParams(const std::string& parameter) {
    std::string color;
    if (ros::param::get("/" + VISUALIZER + "/" + parameter, color)) return getColorFromString(color);
    else throw std::runtime_error("Visualizer node requires '" + parameter + "' color parameter!");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, VISUALIZER);
    bool flatten;
    ros::param::param<bool>("/" + VISUALIZER + "/" + FLATTEN_OUTPUT, flatten, true);
    Color bottomBackground = readColorFromParams(BOTTOM_BACKGROUND_COLOR);
    Color topBackground = readColorFromParams(TOP_BACKGROUND_COLOR);
    Color centerMarker = readColorFromParams(CENTER_MARKER_COLOR);
    Color algorithmicDetection = readColorFromParams(ALGORITHMIC_DETECTION_COLOR);
    Visualizer bsObject(bottomBackground, topBackground, centerMarker, algorithmicDetection, flatten);
    return 0;
}
