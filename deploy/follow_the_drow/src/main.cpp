#include "main.hpp"

#include "ros/ros.h"

#include <functional>
#include <stdexcept>


int HEARTBEAT_RATE;
const std::string HEARTBEAT_RATE_NAME = "heartbeat_rate";
std::string RAW_DATA_TOPIC;
const std::string RAW_DATA_TOPIC_NAME = "raw_data_topic";
std::string ALGORITHMIC_DETECTOR_TOPIC;
const std::string ALGORITHMIC_DETECTOR_TOPIC_NAME = "algorithmic_detector_topic";
std::string DROW_DETECTOR_TOPIC;
const std::string DROW_DETECTOR_TOPIC_NAME = "drow_detector_topic";

std::map<std::string, double> TRANSFORM_DATA;
const std::string TRANSFORM_DATA_NAME = "transform_data";
std::string TOP_LIDAR_TOPIC;
const std::string TOP_LIDAR_TOPIC_NAME = "top_lidar_topic";
std::string BOTTOM_LIDAR_TOPIC;
const std::string BOTTOM_LIDAR_TOPIC_NAME = "bottom_lidar_topic";
std::string ODOMETRY_TOPIC;
const std::string ODOMETRY_TOPIC_NAME = "odometry_topic";

std::string GENERAL_FRAME;
const std::string GENERAL_FRAME_NAME = "general_frame";
bool FLATTEN_OUTPUT;
const std::string FLATTEN_OUTPUT_NAME = "flatten_output";
Color BOTTOM_BACKGROUND_COLOR;
const std::string BOTTOM_BACKGROUND_COLOR_NAME = "bottom_backgorund_color";
Color TOP_BACKGROUND_COLOR;
const std::string TOP_BACKGROUND_COLOR_NAME = "top_backgorund_color";
Color CENTER_MARKER_COLOR;
const std::string CENTER_MARKER_COLOR_NAME = "center_marker_color";
Color ALGORITHMIC_DETECTION_COLOR;
const std::string ALGORITHMIC_DETECTION_COLOR_NAME = "algorithmic_detection_color";
Color DROW_DETECTION_COLOR;
const std::string DROW_DETECTION_COLOR_NAME = "drow_detection_color";
std::string BACK_VISUALIZATION_TOPIC;
const std::string BACK_VISUALIZATION_TOPIC_NAME = "back_visualization_topic";
std::string FRONT_VISUALIZATION_TOPIC;
const std::string FRONT_VISUALIZATION_TOPIC_NAME = "front_visualization_topic";

follow_the_drow::DetectorType ALGORITHMIC_DETECOR_TYPE;
const std::string ALGORITHMIC_DETECOR_TYPE_NAME = "algorithmic_detector_type";
bool ALGORITHMIC_DETECOR_VERBOSE;
const std::string ALGORITHMIC_DETECOR_VERBOSE_NAME = "algorithmic_detector_verbose";


template <typename T> T readFromParams(const std::string parameter) {
    T instance;
    if (ros::param::get(parameter, instance)) return instance;
    else throw std::runtime_error("Parameter '" + parameter + "' not defined!");
}

template <typename T> T readFromParams(const std::string parameter, std::function<T(std::string)> converter) {
    return converter(readFromParams<std::string>(parameter));
}

void loadArgumentsForNode(int argc, char** argv, const std::string& name) {
    ros::init(argc, argv, name);
    HEARTBEAT_RATE = readFromParams<int>("/" + HEARTBEAT_RATE_NAME);
    RAW_DATA_TOPIC = readFromParams<std::string>("/" + RAW_DATA_TOPIC_NAME);
    ALGORITHMIC_DETECTOR_TOPIC = readFromParams<std::string>("/" + ALGORITHMIC_DETECTOR_TOPIC_NAME);
    DROW_DETECTOR_TOPIC = readFromParams<std::string>("/" + DROW_DETECTOR_TOPIC_NAME);
    std::string prefix = "/" + name + "/";

    if (name == LIVE_LOADER) {
        TRANSFORM_DATA = readFromParams<std::map<std::string, double>>(prefix + TRANSFORM_DATA_NAME);
        TOP_LIDAR_TOPIC = readFromParams<std::string>(prefix + TOP_LIDAR_TOPIC_NAME);
        BOTTOM_LIDAR_TOPIC = readFromParams<std::string>(prefix + BOTTOM_LIDAR_TOPIC_NAME);
        ODOMETRY_TOPIC = readFromParams<std::string>(prefix + ODOMETRY_TOPIC_NAME);
    } else if (name == VISUALIZER) {
        GENERAL_FRAME = readFromParams<std::string>(prefix + GENERAL_FRAME_NAME);
        FLATTEN_OUTPUT = readFromParams<bool>(prefix + FLATTEN_OUTPUT_NAME);
        BOTTOM_BACKGROUND_COLOR = readFromParams<Color>(prefix + BOTTOM_BACKGROUND_COLOR_NAME, &getColorFromString);
        TOP_BACKGROUND_COLOR = readFromParams<Color>(prefix + TOP_BACKGROUND_COLOR_NAME, &getColorFromString);
        CENTER_MARKER_COLOR = readFromParams<Color>(prefix + CENTER_MARKER_COLOR_NAME, &getColorFromString);
        ALGORITHMIC_DETECTION_COLOR = readFromParams<Color>(prefix + ALGORITHMIC_DETECTION_COLOR_NAME, &getColorFromString);
        DROW_DETECTION_COLOR = readFromParams<Color>(prefix + DROW_DETECTION_COLOR_NAME, &getColorFromString);
        BACK_VISUALIZATION_TOPIC = readFromParams<std::string>(prefix + BACK_VISUALIZATION_TOPIC_NAME);
        FRONT_VISUALIZATION_TOPIC = readFromParams<std::string>(prefix + FRONT_VISUALIZATION_TOPIC_NAME);
    } else if (name == ALGORITHMIC_DETECTOR) {
        ALGORITHMIC_DETECOR_TYPE = readFromParams<follow_the_drow::DetectorType>(prefix + ALGORITHMIC_DETECOR_TYPE_NAME, &follow_the_drow::getDetectorTypeFromString);
        ALGORITHMIC_DETECOR_VERBOSE = readFromParams<bool>(prefix + ALGORITHMIC_DETECOR_VERBOSE_NAME);
    } else throw std::runtime_error("Unknown node name '" + name + "'!");
}
