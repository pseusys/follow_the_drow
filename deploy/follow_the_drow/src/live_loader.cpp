#include "live_loader.hpp"

#include "follow_the_drow/raw_data.h"

#include "main.hpp"

#define POLAR 0
#define CARTESIAN 1


std::vector<geometry_msgs::Point> LiveLoader::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::array<geometry_msgs::Point, 2>& transform) const {
    int nbBeams = ((-1 * scan->angle_min) + scan->angle_max) / scan->angle_increment;
    std::vector<geometry_msgs::Point> latestScan(nbBeams);

    float beamAngle = scan->angle_min;
    for (int loop = 0; loop < nbBeams; loop++, beamAngle += scan->angle_increment) {
        bool measureOutOfRange = (scan->ranges[loop] < scan->range_max) && (scan->ranges[loop] > scan->range_min);
        float patchedMeasure = measureOutOfRange ? scan->ranges[loop] : scan->range_max;

        latestScan[loop].x = transform[POLAR].x + transform[CARTESIAN].x + patchedMeasure;
        latestScan[loop].y = transform[POLAR].y + transform[CARTESIAN].y + beamAngle;
        latestScan[loop].z = transform[POLAR].z + transform[CARTESIAN].z;
    }
    return latestScan;
}

void LiveLoader::bottomLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    latestBottomScan = lidarCallback(scan, bottomLidarTransform);
    bottomLidarInitialized = true;
}

void LiveLoader::topLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    latestTopScan = lidarCallback(scan, topLidarTransform);
    topLidarInitialized = true;
}

void LiveLoader::update() const {
    if (!bottomLidarInitialized || !topLidarInitialized) return;
    follow_the_drow::raw_data message;
    message.bottom_lidar = latestBottomScan;
    message.top_lidar = latestTopScan;
    rawData.publish(message);
}

LiveLoader::LiveLoader() {
    bottomLidar = handle.subscribe(BOTTOM_LASER_TOPIC, 1, &LiveLoader::bottomLidarCallback, this);
    topLidar = handle.subscribe(TOP_LASER_TOPIC, 1, &LiveLoader::topLidarCallback, this);
    rawData = handle.advertise<follow_the_drow::raw_data>(RAW_DATA_TOPIC, 1);

    ros::Rate rate(HEARTBEAT_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        update();
        rate.sleep();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, LIVE_LOADER);
    LiveLoader bsObject;
    ros::spin();
    return 0;
}
