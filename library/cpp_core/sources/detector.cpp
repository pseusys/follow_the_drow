#include "detectors.hpp"

using namespace follow_the_drow;


const std::vector<Point> AbstractDetector::forward(const std::vector<Point>& latestBottomScan) {
    Point odometry;
    std::vector<Point> latestTopScan;
    return forward(latestBottomScan, latestTopScan, false, odometry, false);
}

const std::vector<Point> AbstractDetector::forward(const std::vector<Point>& latestBottomScan, const Point& odometry) {
    std::vector<Point> latestTopScan;
    return forward(latestBottomScan, latestTopScan, false, odometry, true);
}

const std::vector<Point> AbstractDetector::forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan) {
    Point odometry;
    bool topScanExists = latestTopScan.size() > 0;
    return forward(latestBottomScan, latestTopScan, topScanExists, odometry, false);
}

const std::vector<Point> AbstractDetector::forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan, const Point& odometry) {
    bool topScanExists = latestTopScan.size() > 0;
    return forward(latestBottomScan, latestTopScan, topScanExists, odometry, true);
}

AbstractDetector::AbstractDetector(bool verbose): logging(verbose) {}
