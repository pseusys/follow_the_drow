#pragma once

#include <string>

#define HEARTBEAT_RATE 10


const std::string GENERAL_FRAME = "base_link";

const std::string TOP_LASER_TOPIC = "scan2";
const std::string BOTTOM_LASER_TOPIC = "scan";

const std::string RAW_DATA_TOPIC = "raw_data";
const std::string ALGORITHMIC_DETECTOR_TOPIC = "algorithmic_detector";
const std::string BACK_VISUALIZATION_TOPIC = "visualization_back";
const std::string FRONT_VISUALIZATION_TOPIC = "visualization_front";

const std::string LIVE_LOADER = "follow_the_drow_live_loader";
const std::string VISUALIZER = "follow_the_drow_visualizer";

const std::string FLATTEN_OUTPUT = "flatten_output";
const std::string BOTTOM_BACKGROUND_COLOR = "bottom_backgorund_color";
const std::string TOP_BACKGROUND_COLOR = "top_backgorund_color";
const std::string CENTER_MARKER_COLOR = "center_marker_color";
const std::string ALGORITHMIC_DETECTION_COLOR = "algorithmic_detection_color";
