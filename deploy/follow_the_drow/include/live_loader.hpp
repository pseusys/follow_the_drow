#pragma once

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include "main.hpp"
#include "transformation.hpp"


class LiveLoader {
    private:
        ros::NodeHandle handle;
        ros::Subscriber bottom_lidar, top_lidar;
        ros::Publisher raw_data;

        std::array<geometry_msgs::Point, 2> bottom_lidar_transform {
            bottom_laser_transform_translation(),
            bottom_laser_transform_rotation()
        };
        std::array<geometry_msgs::Point, 2> top_lidar_transform {
            top_laser_transform_translation(),
            top_laser_transform_rotation()
        };

        bool bottom_lidar_initialized = false;
        bool top_lidar_initialized = false;

        std::vector<geometry_msgs::Point> latest_bottom_scan;
        std::vector<geometry_msgs::Point> latest_top_scan;

        std::vector<geometry_msgs::Point> lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan, std::array<geometry_msgs::Point, 2>& transform) const;
        void bottom_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void top_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void update();

    public:
        LiveLoader();
};

