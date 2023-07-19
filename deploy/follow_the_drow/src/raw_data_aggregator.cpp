#include <stdexcept>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include "follow_the_drow/raw_data.h"
#include "transformation.hpp"
#include "main.hpp"


#define polar 0
#define cartesian 1


class raw_data_aggregator {
    private:
        ros::NodeHandle handle;
        ros::Subscriber bottom_lidar, top_lidar;
        ros::Publisher raw_data;

        // Transforms for different devices.
        std::array<geometry_msgs::Point, 2> bottom_lidar_transform {
            bottom_laser_transform_translation(),
            bottom_laser_transform_rotation()
        };
        std::array<geometry_msgs::Point, 2> top_lidar_transform {
            top_laser_transform_translation(),
            top_laser_transform_rotation()
        };

        // Shouldn't send aggregated data untill all callbacks are received at lease once.
        bool bottom_lidar_initialized = false;
        bool top_lidar_initialized = false;

        // Latest received from devices data: 2 arrays for each lidar, polar (0) and cortesian (1).
        std::array<std::vector<geometry_msgs::Point>, 2> latest_bottom_scan {
            std::vector<geometry_msgs::Point>(MAX_LIDAR_SCAN_COUNT),
            std::vector<geometry_msgs::Point>(MAX_LIDAR_SCAN_COUNT)
        };
        std::array<std::vector<geometry_msgs::Point>, 2> latest_top_scan {
            std::vector<geometry_msgs::Point>(MAX_LIDAR_SCAN_COUNT),
            std::vector<geometry_msgs::Point>(MAX_LIDAR_SCAN_COUNT)
        };

        void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan, std::array<std::vector<geometry_msgs::Point>, 2>& storage, std::array<geometry_msgs::Point, 2>& transform) {
            int nb_beams = ((-1 * scan->angle_min) + scan->angle_max) / scan->angle_increment;

            float beam_angle = scan->angle_min + scan->angle_increment * OUT_OF_VIEW_MEASURES;
            for (int lp = OUT_OF_VIEW_MEASURES; lp < nb_beams - OUT_OF_VIEW_MEASURES; lp++, beam_angle += scan->angle_increment) {
                int loop = lp - OUT_OF_VIEW_MEASURES;
                bool measure_out_of_range = (scan->ranges[lp] < scan->range_max) && (scan->ranges[lp] > scan->range_min);
                float patched_measure = measure_out_of_range ? scan->ranges[lp] : scan->range_max;

                // Calculate polar coordinates.
                storage[polar][loop].x = transform[polar].x + transform[cartesian].x + patched_measure;
                storage[polar][loop].y = transform[polar].y + transform[cartesian].y + beam_angle;
                storage[polar][loop].z = transform[polar].z + transform[cartesian].z;

                // Calculate cartesian coordinates.
                storage[cartesian][loop] = polar_to_cartesian(storage[polar][loop]);
            }
        }

        void bottom_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
            lidar_callback(scan, latest_bottom_scan, bottom_lidar_transform);
            bottom_lidar_initialized = true;
        }

        void top_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
            lidar_callback(scan, latest_top_scan, top_lidar_transform);
            top_lidar_initialized = true;
        }

        void update() {
            if (!bottom_lidar_initialized || !top_lidar_initialized) return;
            follow_the_drow::raw_data message;
            message.bottom_lidar_polar = latest_bottom_scan[polar];
            message.bottom_lidar_cartesian = latest_bottom_scan[cartesian];
            message.top_lidar_polar = latest_top_scan[polar];
            message.top_lidar_cartesian = latest_top_scan[cartesian];
            raw_data.publish(message);
        }

    public:
        raw_data_aggregator() {
            bottom_lidar = handle.subscribe("scan", 1, &raw_data_aggregator::bottom_lidar_callback, this);
            top_lidar = handle.subscribe("scan2", 1, &raw_data_aggregator::top_lidar_callback, this);
            raw_data = handle.advertise<follow_the_drow::raw_data>(raw_data_topic, 1);

            ros::Rate r(HEARTBEAT_RATE);

            while (ros::ok()) {
                ros::spinOnce();
                update();
                r.sleep();
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "aggregator");

    ROS_INFO("waiting for activation of raw data aggregator");
    raw_data_aggregator bsObject;

    ros::spin();
    return 0;
}
