#include "detectors.hpp"

using namespace follow_the_drow;


const std::vector<Point> AbstractDetector::forward(const std::vector<Point>& latestBottomScan) {
    std::vector<Point> latestTopScan;
    return forward(latestBottomScan, latestTopScan, false);
}

const std::vector<Point> AbstractDetector::forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan) {
    bool topScanExists = latestTopScan.size() > 0;
    return forward(latestBottomScan, latestTopScan, topScanExists);
}
