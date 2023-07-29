#pragma once

#include <map>
#include <string>

#include "follow_the_drow/detector_factory.hpp"

#include "colors.hpp"


// Node names
const std::string LIVE_LOADER = "follow_the_drow_live_loader";
const std::string FILE_LOADER = "follow_the_drow_file_loader";
const std::string VISUALIZER = "follow_the_drow_visualizer";
const std::string ALGORITHMIC_DETECTOR = "follow_the_drow_algorithmic_detector";
const std::string DROW_DETECTOR = "follow_the_drow_drow_detector";

// General arguments
extern int HEARTBEAT_RATE;
extern std::string RAW_DATA_TOPIC;
extern std::string ANNOTATED_DATA_TOPIC;
extern std::string ALGORITHMIC_DETECTOR_TOPIC;
extern std::string DROW_DETECTOR_TOPIC;

void loadArgumentsForNode(int argc, char** argv, const std::string& name);

// Live loader node arguments
extern std::map<std::string, float> TRANSFORM_DATA;
extern std::string TOP_LIDAR_TOPIC;
extern std::string BOTTOM_LIDAR_TOPIC;
extern std::string ODOMETRY_TOPIC;

// Visualizer node arguments
extern std::string GENERAL_FRAME;
extern bool FLATTEN_OUTPUT;
extern Color BOTTOM_BACKGROUND_COLOR;
extern Color TOP_BACKGROUND_COLOR;
extern Color ANNOTATED_BACKGROUND_COLOR;
extern Color CENTER_MARKER_COLOR;
extern Color ALGORITHMIC_DETECTION_COLOR;
extern Color DROW_DETECTION_COLOR;
extern std::string BACK_VISUALIZATION_TOPIC;
extern std::string FRONT_VISUALIZATION_TOPIC;

// Algorithmic detector node arguments
extern follow_the_drow::DetectorType ALGORITHMIC_DETECOR_TYPE;
extern bool ALGORITHMIC_DETECOR_VERBOSE;
