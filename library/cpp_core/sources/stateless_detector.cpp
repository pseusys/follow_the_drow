// Person detector using 2 lidar data
// Written by O. Aycard

// TODO: something pythonic?
#include <iostream>

#include "detectors.hpp"

using namespace follow_the_drow;


const std::vector<Cluster> StatelessDetector::performClustering(const std::vector<Point>& storage) const {
    std::vector<Cluster> clusters;
    int clusterStart = 0;
    for (int loop = 1; loop < storage.size(); loop++)
        if (storage[loop-1].distance_to(storage[loop]) >= CLUSTER_THRESHOLD) {
            clusters.push_back(Cluster(clusterStart, loop - 1, storage));
            clusterStart = loop;
        }
    if (clusterStart != storage.size() - 1)
        clusters.push_back(Cluster(clusterStart, storage.size() - 1, storage));
    return clusters;
}

const std::vector<Cluster> StatelessDetector::detectLegs(const std::vector<Cluster>& clusters) const {
    std::vector<Cluster> legClusters;
    for (int loop = 0; loop < clusters.size(); loop++)
        if ((clusters[loop].size() < LEG_SIZE_MAX) && (clusters[loop].size() > LEG_SIZE_MIN))
            legClusters.push_back(Cluster(clusters[loop]));
    if (legClusters.size() > 0) std::cout << ("%ld legs have been detected", legClusters.size()) << std::endl;
    return legClusters;
}

const std::vector<Cluster> StatelessDetector::detectChests(const std::vector<Cluster>& clusters) const {
    std::vector<Cluster> chestClusters;
    for (int loop = 0; loop < clusters.size(); loop++)
        if ((clusters[loop].size() < CHEST_SIZE_MAX) && (clusters[loop].size() > CHEST_SIZE_MIN))
            chestClusters.push_back(Cluster(clusters[loop]));
    if (chestClusters.size()) std::cout << ("%ld chests have been detected", chestClusters.size()) << std::endl;
    return chestClusters;
}

const std::vector<Point> StatelessDetector::detectPeople(const std::vector<Cluster>& legClusters, const std::vector<Cluster>& chestClusters) const {
    std::vector<Point> detection;

    for (int loopLeftLeg = 0; loopLeftLeg < legClusters.size(); loopLeftLeg++)
        for (int loopRightLeg = loopLeftLeg + 1; loopRightLeg < legClusters.size(); loopRightLeg++) {
            double twoLegsDistance = legClusters[loopLeftLeg].distance_to(legClusters[loopRightLeg]);
            bool personLegsDetected = (twoLegsDistance > LEGS_DISTANCE_MIN && twoLegsDistance < LEGS_DISTANCE_MAX);
            if (!personLegsDetected) continue;
            
            for (int loopChest = 0; loopChest < chestClusters.size(); loopChest++) {
                double leftChestDistance = legClusters[loopLeftLeg].distance_to(chestClusters[loopChest]);
                double rightChestDistance = legClusters[loopRightLeg].distance_to(chestClusters[loopChest]);
                bool personChestDetected = (leftChestDistance < DISTANCE_LEVEL) && (rightChestDistance < DISTANCE_LEVEL);
                if (!personChestDetected) continue;

                detection.push_back(chestClusters[loopChest].betweenPoint());
            }
        }
    return detection;
}

const std::vector<Point> StatelessDetector::forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan) {
    const std::vector<Cluster>& legs = detectLegs(performClustering(latestBottomScan));
    const std::vector<Cluster>& chests = detectChests(performClustering(latestTopScan));
    return detectPeople(legs, chests);
}

StatelessDetector::StatelessDetector() {}
