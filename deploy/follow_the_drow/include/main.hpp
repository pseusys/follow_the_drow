#ifndef DETECTION_MAIN_HPP
#define DETECTION_MAIN_HPP

#define HEARTBEAT_RATE 10
#define OUT_OF_VIEW_MEASURES 300
#define MAX_LIDAR_SCAN_COUNT 125

const char* general_frame = "base_link";

const char* raw_data_topic = "raw_data";
const char* algorithmic_detector_topic = "algorithmic_detector";
const char* computer_vision_topic = "cv_detector";
const char* back_visualization_topic = "visualization_back";
const char* front_visualization_topic = "visualization_front";

#endif // DETECTION_MAIN_HPP
