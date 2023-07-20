#include "live_loader.hpp"

#include "follow_the_drow/raw_data.h"

#define polar 0
#define cartesian 1


std::vector<geometry_msgs::Point> LiveLoader::lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan, std::array<geometry_msgs::Point, 2>& transform) const {
    int nb_beams = ((-1 * scan->angle_min) + scan->angle_max) / scan->angle_increment;
    std::vector<geometry_msgs::Point> latest_scan(nb_beams);

    float beam_angle = scan->angle_min;
    for (int loop = 0; loop < nb_beams; loop++, beam_angle += scan->angle_increment) {
        bool measure_out_of_range = (scan->ranges[loop] < scan->range_max) && (scan->ranges[loop] > scan->range_min);
        float patched_measure = measure_out_of_range ? scan->ranges[loop] : scan->range_max;

        latest_scan[loop].x = transform[polar].x + transform[cartesian].x + patched_measure;
        latest_scan[loop].y = transform[polar].y + transform[cartesian].y + beam_angle;
        latest_scan[loop].z = transform[polar].z + transform[cartesian].z;
    }
    return latest_scan;
}

void LiveLoader::bottom_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    latest_bottom_scan = lidar_callback(scan, bottom_lidar_transform);
    bottom_lidar_initialized = true;
}

void LiveLoader::top_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    latest_top_scan = lidar_callback(scan, top_lidar_transform);
    top_lidar_initialized = true;
}

void LiveLoader::update() {
    if (!bottom_lidar_initialized || !top_lidar_initialized) return;
    follow_the_drow::raw_data message;
    message.bottom_lidar = latest_bottom_scan;
    message.top_lidar = latest_top_scan;
    raw_data.publish(message);
}

LiveLoader::LiveLoader() {
    bottom_lidar = handle.subscribe(bottom_laser_topic, 1, &LiveLoader::bottom_lidar_callback, this);
    top_lidar = handle.subscribe(top_laser_topic, 1, &LiveLoader::top_lidar_callback, this);
    raw_data = handle.advertise<follow_the_drow::raw_data>(raw_data_topic, 1);

    ros::Rate rate(HEARTBEAT_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        update();
        rate.sleep();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, live_loader);
    LiveLoader bsObject;
    ros::spin();
    return 0;
}
