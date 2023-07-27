#include "algorithmic_detector.hpp"

#include "follow_the_drow/detectors.hpp"
#include "follow_the_drow/detection.h"

#include "main.hpp"
#include "utils.hpp"


void AlgorithmicDetector::rawDataCallback(const follow_the_drow::raw_data::ConstPtr& data) {
    latestBottomScan = std::vector<follow_the_drow::Point>(data->bottom_lidar.size());
    for (int i = 0; i < data->bottom_lidar.size(); i++)
        latestBottomScan[i] = geometryToPoint(polarToCartesian(data->bottom_lidar[i]));

    latestTopScan = std::vector<follow_the_drow::Point>(data->top_lidar.size());
    for (int i = 0; i < data->top_lidar.size(); i++)
        latestTopScan[i] = geometryToPoint(polarToCartesian(data->top_lidar[i]));

   rawDataInitialized = true;
}

void AlgorithmicDetector::update() const {
    if (!rawDataInitialized) return;
    std::vector<follow_the_drow::Point> points = detector->forward(latestBottomScan, latestTopScan);
    std::vector<geometry_msgs::Point> geometries = std::vector<geometry_msgs::Point>(points.size());
    for (int i = 0; i < points.size(); i++) geometries[i] = pointToGeometry(points[i]);

    follow_the_drow::detection detection;
    detection.detection = geometries;
    detectionData.publish(detection);
}

AlgorithmicDetector::AlgorithmicDetector(const follow_the_drow::DetectorType detector, const bool verbose): DetectorFactory(detector, verbose) {
    rawData = handle.subscribe(RAW_DATA_TOPIC, 1, &AlgorithmicDetector::rawDataCallback, this);
    detectionData = handle.advertise<follow_the_drow::detection>(ALGORITHMIC_DETECTOR_TOPIC, 1);

    ros::Rate rate(HEARTBEAT_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        update();
        rate.sleep();
    }
}


int main(int argc, char **argv) {
    loadArgumentsForNode(argc, argv, ALGORITHMIC_DETECTOR);
    AlgorithmicDetector bsObject(ALGORITHMIC_DETECOR_TYPE, ALGORITHMIC_DETECOR_VERBOSE);
    ros::spin();
    return 0;
}
