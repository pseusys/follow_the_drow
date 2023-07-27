// Person detector using 2 lidar data
// Written by O. Aycard

// TODO: something pythonic?
#include "detectors.hpp"

using namespace follow_the_drow;


const std::vector<Cluster> StatelessDetector::performClustering(const std::vector<Point>& storage) const {
    std::vector<Cluster> clusters;
    int clusterStart = 0;
    for (int loop = 1; loop < storage.size(); loop++)
        if (storage[loop-1].distanceTo(storage[loop]) >= CLUSTER_THRESHOLD) {
            clusters.push_back(Cluster(clusterStart, loop - 1, storage));
            clusterStart = loop;
        }
    if (clusterStart != storage.size() - 1 && storage.size() > 0)
        clusters.push_back(Cluster(clusterStart, storage.size() - 1, storage));
    return clusters;
}

const std::vector<Cluster> StatelessDetector::detectLegs(const std::vector<Cluster>& clusters) const {
    std::vector<Cluster> legClusters;
    for (int loop = 0; loop < clusters.size(); loop++)
        if ((clusters[loop].size() < LEG_SIZE_MAX) && (clusters[loop].size() > LEG_SIZE_MIN))
            legClusters.push_back(Cluster(clusters[loop]));
    if (logging && legClusters.size() > 0) LOG("%ld legs have been detected\n", legClusters.size());
    return legClusters;
}

const std::vector<Cluster> StatelessDetector::detectChests(const std::vector<Cluster>& clusters) const {
    std::vector<Cluster> chestClusters;
    for (int loop = 0; loop < clusters.size(); loop++)
        if ((clusters[loop].size() < CHEST_SIZE_MAX) && (clusters[loop].size() > CHEST_SIZE_MIN))
            chestClusters.push_back(Cluster(clusters[loop]));
    if (logging && chestClusters.size() > 0) LOG("%ld chests have been detected\n", chestClusters.size());
    return chestClusters;
}

const std::vector<Point> StatelessDetector::detectPeople(const std::vector<Cluster>& legClusters, const std::vector<Cluster>& chestClusters, bool topScanReceived) const {
    std::vector<Point> detection;

    for (int loopLeftLeg = 0; loopLeftLeg < legClusters.size(); loopLeftLeg++)
        for (int loopRightLeg = loopLeftLeg + 1; loopRightLeg < legClusters.size(); loopRightLeg++) {
            double twoLegsDistance = legClusters[loopLeftLeg].distanceTo(legClusters[loopRightLeg]);
            bool personLegsDetected = (twoLegsDistance > LEGS_DISTANCE_MIN && twoLegsDistance < LEGS_DISTANCE_MAX);
            if (!personLegsDetected) continue;

            if (!topScanReceived) {
                detection.push_back(legClusters[loopLeftLeg].middleBetween(legClusters[loopRightLeg]));
                continue;
            }

            for (int loopChest = 0; loopChest < chestClusters.size(); loopChest++) {
                double leftChestDistance = legClusters[loopLeftLeg].distanceTo(chestClusters[loopChest]);
                double rightChestDistance = legClusters[loopRightLeg].distanceTo(chestClusters[loopChest]);
                bool personChestDetected = (leftChestDistance < DISTANCE_LEVEL) && (rightChestDistance < DISTANCE_LEVEL);
                if (!personChestDetected) continue;

                detection.push_back(chestClusters[loopChest].betweenPoint());
            }
        }
    if (logging && detection.size() > 0) LOG("%ld persons have been detected\n", detection.size());
    return detection;
}

const std::vector<Point> StatelessDetector::forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan, bool topScanReceived) {
    std::vector<Cluster> legs = detectLegs(performClustering(latestBottomScan));
    std::vector<Cluster> chests;
    if (topScanReceived) chests = detectChests(performClustering(latestTopScan));
    return detectPeople(legs, chests, topScanReceived);
}

StatelessDetector::StatelessDetector(bool verbose): AbstractDetector(verbose) {}
