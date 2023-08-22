#pragma once

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "follow_the_drow/raw_data.h"
#include "follow_the_drow/detector.hpp"

#include "tracker.hpp"


class AlgorithmicDetector {
    private:
        follow_the_drow::AlgorithmicDetector detector;
        TrackerPolicy tracker;

        ros::NodeHandle handle;
        ros::Subscriber rawData;
        ros::Publisher detectionData, followMeData;

        follow_the_drow::Point person;
        bool personTracked = false;

        std::vector<follow_the_drow::Point> latestBottomScan, latestTopScan;
        follow_the_drow::Point latestOdometry;

        bool rawDataInitialized = false;
        void rawDataCallback(const follow_the_drow::raw_data::ConstPtr& data);

        void update();
    
    public:
        AlgorithmicDetector(int frequencyInit, int frequencyMax, float uncertaintyMax, float uncertaintyMin, float uncertaintyInc, float clusterThreshold, float distanceLevel, float legSizeMin, float legSizeMax, float chestSizeMin, float chestSizeMax, float legsDistanceMin, float legsDistanceMax, TrackerPolicy policy, bool logging);
};
