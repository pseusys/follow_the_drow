#include "algorithmic_detector.hpp"

#include "main.hpp"


void AlgorithmicDetector::initializeScanParams(double minimumAngle, double incrementAngle, double minimumRange, double maximumRange) {

}

void AlgorithmicDetector::bottomLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

}

void AlgorithmicDetector::topLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

}

void AlgorithmicDetector::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom) {

}

void AlgorithmicDetector::update() const {

}

AlgorithmicDetector::AlgorithmicDetector(const follow_the_drow::DetectorType detector): DetectorFactory(detector, -1, -1, -1, -1) {

}


int main(int argc, char **argv) {
    loadArgumentsForNode(argc, argv, ALGORITHMIC_DETECTOR);
    AlgorithmicDetector bsObject(ALGORITHMIC_DETECOR_TYPE);
    ros::spin();
    return 0;
}
