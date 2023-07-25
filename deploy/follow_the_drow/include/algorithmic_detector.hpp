#pragma once

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "follow_the_drow/raw_data.h"
#include "follow_the_drow/detector_factory.hpp"


class AlgorithmicDetector: follow_the_drow::DetectorFactory {
    private:
        ros::NodeHandle handle;
        ros::Subscriber rawData;
        ros::Publisher detectionData;

        std::vector<follow_the_drow::Point> latestBottomScan, latestTopScan;

        bool rawDataInitialized = false;
        void rawDataCallback(const follow_the_drow::raw_data::ConstPtr& data);

        // Convert polar to Points
        // Run forward
        void update() const;
    
    public:
        AlgorithmicDetector(const follow_the_drow::DetectorType detector);
};
