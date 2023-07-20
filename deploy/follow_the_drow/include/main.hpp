#pragma once

#define HEARTBEAT_RATE 10

const char* general_frame = "base_link";

const char* top_laser_topic = "scan2";
const char* bottom_laser_topic = "scan";

const char* raw_data_topic = "raw_data";
const char* algorithmic_detector_topic = "algorithmic_detector";
const char* back_visualization_topic = "visualization_back";
const char* front_visualization_topic = "visualization_front";

const char* live_loader = "Live Data Loader";
const char* visualizer = "All Visualizer";
