#pragma once

#include <vector>

#include "utils.hpp"

#define CLUSTER_THRESHOLD 0.2

#define LEG_SIZE_MIN 0.0
#define LEG_SIZE_MAX 0.2
#define LEGS_DISTANCE_MIN 0.0
#define LEGS_DISTANCE_MAX 0.7

#define CHEST_SIZE_MIN 0.3
#define CHEST_SIZE_MAX 0.8

#define DISTANCE_LEVEL 0.6


class Cluster {
    private:
        const double dSize;
        const int iStart, iMiddle, iEnd;
        const Point pStart, pMiddle, pEnd, pBetween;

    public:
        Cluster(const Cluster& cluster);
        Cluster(int start, int end, const std::vector<Point>& reference);

        double size() const;
        int start() const;
        int middle() const;
        int end() const;

        const Point& startPoint() const;
        const Point& middlePoint() const;
        const Point& betweenPoint() const;
        const Point& endPoint() const;

        double distance_to(const Cluster& cluster) const;
};


class AbstractDetector {
    public:
        virtual const std::vector<Point>& forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan) = 0;
};


class StatelessDetector: public AbstractDetector {
    private:
        const std::vector<Cluster>& performClustering(const std::vector<Point>& storage) const;
        const std::vector<Cluster>& detectLegs(const std::vector<Cluster>& clusters) const;
        const std::vector<Cluster>& detectChests(const std::vector<Cluster>& clusters) const;
        const std::vector<Point>& detectPeople(const std::vector<Cluster>& legClusters, const std::vector<Cluster>& chestClusters) const;
        const std::vector<Point>& forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan) override;

    public:
        StatelessDetector();
};
