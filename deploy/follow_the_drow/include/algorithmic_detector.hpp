#pragma once

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "follow_the_drow/detector_factories.hpp"


class AlgorithmicDetector: follow_the_drow::DetectorFactory {
    private:
        bool scanParamsInitialized = false;
        void initializeScanParams(double minimumAngle, double incrementAngle, double minimumRange, double maximumRange);

        bool bottomLidarInitialized = false;
        void bottomLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        bool topLidarInitialized = false;
        void topLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        bool odometryInitialized = false;
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);

        // Convert polar to Points
        // Run forward
        void update() const;
    
    public:
        AlgorithmicDetector(const follow_the_drow::DetectorType detector);
};
