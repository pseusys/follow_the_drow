// person detector using 2 lidar data
// written by O. Aycard

#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include "detection/raw_data.h"
#include "detection/detection.h"
#include "transformation.hpp"
#include "main.hpp"


//threshold for clustering
#define cluster_threshold 0.2

//used for detection of leg
#define leg_size_min 0.0
#define leg_size_max 0.2
#define legs_distance_min 0.0
#define legs_distance_max 0.7

//used for detection of chest
#define chest_size_min 0.3
#define chest_size_max 0.8

#define distance_level 0.6

#define found 1
#define skipped 0


class measure_cluster {
    private:
        float c_size;
        int c_start {}, c_end {};
        geometry_msgs::Point c_middle;

    public:
        measure_cluster(int start, int end, const std::vector<geometry_msgs::Point>& reference): c_start { start }, c_end { end } {
            c_size = cartesian_distance(reference[c_start], reference[c_end]);
            c_middle.x = (reference[c_start].x + reference[c_end].x) / 2;
            c_middle.y = (reference[c_start].y + reference[c_end].y) / 2;
            c_middle.z = (reference[c_start].z + reference[c_end].z) / 2;
        }

        float size() { return c_size; }
        int start() { return c_start; }
        int end() { return c_end; }
        geometry_msgs::Point middle() { return c_middle; }
};


class body_part_cluster {
    private:
        measure_cluster& c_cluster;

    public:
        body_part_cluster(measure_cluster& cluster): c_cluster { cluster } {};

        measure_cluster& cluster() { return c_cluster; }
        geometry_msgs::Point part() { return c_cluster.middle(); }
};


class algorithmic_detector {
    private:
        ros::NodeHandle handle;
        ros::Subscriber raw_data;
        ros::Publisher algorithmic;

        std::vector<geometry_msgs::Point> latest_bottom_scan, latest_top_scan;
        std::vector<measure_cluster> top_clusters, bottom_clusters;
        std::vector<body_part_cluster> legs, chests;

        bool data_received = false;
        std::vector<unsigned char> detection;

        void perform_clustering(const std::vector<geometry_msgs::Point>& storage, std::vector<measure_cluster>& clusters) {
            clusters.clear();
            int cluster_start = 0;
            for (int loop = 1; loop < storage.size(); loop++)
                if (cartesian_distance(storage[loop-1], storage[loop]) >= cluster_threshold) {
                    clusters.push_back(measure_cluster(cluster_start, loop - 1, storage));
                    cluster_start = loop;
                }
            if (cluster_start != storage.size() - 1)
                clusters.push_back(measure_cluster(cluster_start, storage.size() - 1, storage));
        }

        void detect_legs() {
            legs.clear();
            for (int loop = 0; loop < bottom_clusters.size(); loop++)
                if ((bottom_clusters[loop].size() < leg_size_max) && (bottom_clusters[loop].size() > leg_size_min))
                    legs.push_back(body_part_cluster(bottom_clusters[loop]));
            if (legs.size() > 0) ROS_INFO("%ld legs have been detected", legs.size());
        }

        void detect_chests() {
            chests.clear();
            for (int loop = 0; loop < top_clusters.size(); loop++)
                if ((top_clusters[loop].size() < chest_size_max) && (top_clusters[loop].size() > chest_size_min))
                    chests.push_back(body_part_cluster(top_clusters[loop]));
            if (chests.size()) ROS_INFO("%ld chests have been detected", chests.size());
        }

        void detect_people() {
            for (int ray = 0; ray < MAX_LIDAR_SCAN_COUNT; ray++) detection[ray] = skipped;

            for (int loop_left_leg = 0; loop_left_leg < legs.size(); loop_left_leg++)
                for (int loop_right_leg = loop_left_leg + 1; loop_right_leg < legs.size(); loop_right_leg++) {
                    float two_legs_distance = cartesian_distance(legs[loop_left_leg].part(), legs[loop_right_leg].part());
                    bool person_legs_detected = (two_legs_distance > legs_distance_min && two_legs_distance < legs_distance_max);
                    if (!person_legs_detected) continue;
                    
                    for (int loop_chest = 0; loop_chest < chests.size(); loop_chest++) {
                        float left_chest_distance = cartesian_distance(legs[loop_left_leg].part(), chests[loop_chest].part());
                        float right_chest_distance = cartesian_distance(legs[loop_right_leg].part(), chests[loop_chest].part());
                        bool person_chest_detected = (left_chest_distance < distance_level) && (right_chest_distance < distance_level);
                        if (!person_chest_detected) continue;

                        measure_cluster chest_cluster = chests[loop_chest].cluster();
                        int chest_middle = (chest_cluster.start() + chest_cluster.end()) / 2;
                        detection[chest_middle] = found;
                    }
                }
        }

        void raw_data_callback(const detection::raw_data::ConstPtr& data) {
            data_received = true;
            latest_bottom_scan = data->bottom_lidar_cartesian;
            latest_top_scan = data->top_lidar_cartesian;
        }

        void update() {
            if (data_received) {
                ROS_INFO("detecting people algoritmically");
                perform_clustering(latest_bottom_scan, bottom_clusters);
                perform_clustering(latest_top_scan, top_clusters);
                detect_legs();
                detect_chests();
                detect_people();

                detection::detection message;
                message.detection = detection;
                algorithmic.publish(message);
            }
        }

    public:
        algorithmic_detector() {
            algorithmic = handle.advertise<detection::detection>(algorithmic_detector_topic, 1);
            raw_data = handle.subscribe(raw_data_topic, 1, &algorithmic_detector::raw_data_callback, this);

            detection = std::vector<unsigned char>(MAX_LIDAR_SCAN_COUNT);
            ros::Rate r(HEARTBEAT_RATE);

            while (ros::ok()) {
                ros::spinOnce();
                update();
                r.sleep();
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "algorithmic_detector");

    ROS_INFO("waiting for activation of algorithmic detector");
    algorithmic_detector bsObject;

    ros::spin();
    return 0;
}
