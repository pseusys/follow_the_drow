#include "algorithmic_detector.hpp"

#include "main.hpp"


AlgorithmicDetector::AlgorithmicDetector(const follow_the_drow::DetectorType detector): DetectorFactory(detector, minAngle, incAngle, -1, -1) {}


int main(int argc, char **argv) {
    ros::init(argc, argv, LIVE_LOADER);
    std::string detector;
    follow_the_drow::DetectorType type;
    if (ros::param::get("/" + ALGORITHMIC_DETECTOR + "/" + ALGORITHMIC_DETECOR_TYPE, detector)) type = follow_the_drow::getDetectorTypeFromString(detector);
    else throw std::runtime_error("Visualizer node requires '" + ALGORITHMIC_DETECOR_TYPE + "' detector type parameter!");
    AlgorithmicDetector bsObject(type);
    ros::spin();
    return 0;
}
