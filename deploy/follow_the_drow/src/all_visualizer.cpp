#include <algorithm>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"

#include "detection/raw_data.h"
#include "detection/detection.h"
#include "main.hpp"


#define flatten_output true


class all_visualizer {
    private:
        ros::NodeHandle handle;
        ros::Subscriber raw_data, algorithmic_detector;
        ros::Publisher back_visualizer, front_visualizer;

        // Latest received from devices data.
        std::vector<geometry_msgs::Point> latest_bottom_scan;
        std::vector<geometry_msgs::Point> latest_top_scan;

        std::vector<geometry_msgs::Point> algorithmic_detector_data;

        bool scan_received = false;
        bool algorithmic_received = false;

        void raw_data_callback(const detection::raw_data::ConstPtr& data) {
            latest_bottom_scan = data->bottom_lidar_cartesian;
            latest_top_scan = data->top_lidar_cartesian;
            scan_received = true;
        }

        void algorithmic_detector_callback(const detection::detection::ConstPtr& data) {
            algorithmic_detector_data.clear();
            for (int i = 0; i < data->detection.size(); i++)
                if (data->detection[i])
                    algorithmic_detector_data.push_back(latest_top_scan[i]);
            algorithmic_received = true;
        }

        void add_point_to_marker(visualization_msgs::Marker& marker, geometry_msgs::Point& point, std_msgs::ColorRGBA& color) {
            if (flatten_output) point.z = 0;
            marker.points.push_back(point);
            marker.colors.push_back(color);
        }

        void add_point_to_marker(visualization_msgs::Marker& marker, geometry_msgs::Point& point, float r, float g, float b) {
            std_msgs::ColorRGBA color;
            color.r = r;
            color.g = g;
            color.b = b;
            color.a = 1;
            add_point_to_marker(marker, point, color);
        }

        void add_point_to_marker(visualization_msgs::Marker& marker, float x, float y, float z, float r, float g, float b) {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            add_point_to_marker(marker, point, r, g, b);
        }

        void update() {
            if (!scan_received) return;
            visualization_msgs::Marker back_marker;

            back_marker.header.frame_id = general_frame;
            back_marker.header.stamp = ros::Time::now();
            back_marker.ns = back_visualization_topic;
            back_marker.id = 0;
            back_marker.type = visualization_msgs::Marker::POINTS;
            back_marker.action = visualization_msgs::Marker::ADD;

            back_marker.pose.orientation.w = 1;
            back_marker.scale.x = 0.05;
            back_marker.scale.y = 0.05;
            back_marker.color.a = 1.0;

            int laser_size = std::min(latest_bottom_scan.size(), latest_top_scan.size());
            for (int loop = 0; loop < laser_size; loop++) {
                add_point_to_marker(back_marker, latest_bottom_scan[loop], 0, 0, 1);
                add_point_to_marker(back_marker, latest_top_scan[loop], 1, 0, 0);
            }

            back_visualizer.publish(back_marker);

            if (!algorithmic_received) return;
            visualization_msgs::Marker front_marker;

            front_marker.header.frame_id = general_frame;
            front_marker.header.stamp = ros::Time::now();
            front_marker.ns = back_visualization_topic;
            front_marker.id = 0;
            front_marker.type = visualization_msgs::Marker::POINTS;
            front_marker.action = visualization_msgs::Marker::ADD;

            front_marker.pose.orientation.w = 1;
            front_marker.scale.x = 0.15;
            front_marker.scale.y = 0.15;
            front_marker.color.a = 1.0;

            for (int loop = 0; loop < algorithmic_detector_data.size(); loop++) {
                add_point_to_marker(front_marker, algorithmic_detector_data[loop], 0, 1, 0);
            }

            add_point_to_marker(front_marker, 0, 0, 0, 1, 0, 1);
            front_visualizer.publish(front_marker);
        }

    public:
        all_visualizer() {
            back_visualizer = handle.advertise<visualization_msgs::Marker>(back_visualization_topic, 1);
            front_visualizer = handle.advertise<visualization_msgs::Marker>(front_visualization_topic, 1);
            raw_data = handle.subscribe(raw_data_topic, 1, &all_visualizer::raw_data_callback, this);
            algorithmic_detector = handle.subscribe(algorithmic_detector_topic, 1, &all_visualizer::algorithmic_detector_callback, this);

            ros::Rate r(HEARTBEAT_RATE);

            while (ros::ok()) {
                ros::spinOnce();
                update();
                r.sleep();
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "visualizer");

    ROS_INFO("waiting for activation of all visualizer");
    all_visualizer bsObject;

    ros::spin();
    return 0;
}
