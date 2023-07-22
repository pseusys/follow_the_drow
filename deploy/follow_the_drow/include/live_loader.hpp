#pragma once

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "transformation.hpp"


class LiveLoader {
    private:
        ros::NodeHandle handle;
        ros::Subscriber bottomLidar, topLidar, odometry;
        ros::Publisher rawData;

        std::array<geometry_msgs::Point, 2> bottomLidarTransform {
            bottomLaserTransformTranslation(),
            bottomLaserTransformRotation()
        };
        std::array<geometry_msgs::Point, 2> topLidarTransform {
            topLaserTransformTranslation(),
            topLaserTransformRotation()
        };

        bool bottomLidarInitialized = false;
        bool topLidarInitialized = false;
        bool odometryInitialized = false;

        std::vector<geometry_msgs::Point> latestBottomScan, latestTopScan;
        geometry_msgs::Point latestOdometry;

        std::vector<geometry_msgs::Point> lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::array<geometry_msgs::Point, 2>& transform) const;
        void bottomLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void topLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void update() const;

    public:
        LiveLoader();
};

