#pragma once

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "follow_the_drow/raw_data.h"
#include "follow_the_drow/detector.hpp"


class AlgorithmicDetector {
    private:
        follow_the_drow::AlgorithmicDetector detector;

        ros::NodeHandle handle;
        ros::Subscriber rawData;
        ros::Publisher detectionData;

        std::vector<follow_the_drow::Point> latestBottomScan, latestTopScan;
        follow_the_drow::Point latestOdometry;

        bool rawDataInitialized = false;
        void rawDataCallback(const follow_the_drow::raw_data::ConstPtr& data);

        void update();
    
    public:
        AlgorithmicDetector(bool logging);
};
