#pragma once

#include <vector>

#include "tracked.hpp"


namespace follow_the_drow {
    class Cluster {
        private:
            float dSize;
            int iStart, iMiddle, iEnd;
            Point pStart, pMiddle, pEnd, pBetween;

        public:
            Cluster(const Cluster& cluster);
            Cluster(int start, int end, const std::vector<Point>& reference);

            float size() const;
            int start() const;
            int middle() const;
            int end() const;

            const Point& startPoint() const;
            const Point& middlePoint() const;
            const Point& betweenPoint() const;
            const Point& endPoint() const;

            float distanceTo(const Cluster& cluster) const;
            Point middleBetween(const Cluster& cluster) const;
    };


    class AlgorithmicDetector {
        private:
            const bool verbose = true;

            Point previousPosition;
            std::vector<Tracked> previousPeople;

            const std::vector<Cluster> performClustering(const std::vector<Point>& storage) const;
            const std::vector<Cluster> detectLegs(const std::vector<Cluster>& clusters) const;
            const std::vector<Cluster> detectChests(const std::vector<Cluster>& clusters) const;
            const std::vector<Point> detectPeople(const std::vector<Cluster>& legClusters, const std::vector<Cluster>& chestClusters, bool topScanReceived) const;

            const std::vector<Tracked> estimatePreviousPeople(const Point& newOdometry);
            const std::vector<Tracked> pullAssociatedPeople(std::vector<Point>& detectedPeople, const std::vector<Tracked>& estimatedPrevious);
            const std::vector<Tracked> trackDetectedPeople(const std::vector<Point>& detectedPeople);
            const std::vector<Point> getCurrentlyDetectedPeople();

            const std::vector<Point> forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan, bool topScanReceived, const Point& odometry, bool odometryReceived);

        public:
            AlgorithmicDetector(bool logging);

            const std::vector<Point> forward(const std::vector<Point>& latestBottomScan);
            const std::vector<Point> forward(const std::vector<Point>& latestBottomScan, const Point& odometry);
            const std::vector<Point> forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan);
            const std::vector<Point> forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan, const Point& odometry);
    };
}
