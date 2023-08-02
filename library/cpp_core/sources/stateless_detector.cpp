// Person detector using 2 lidar data
// Written by O. Aycard

#include <cmath>

#include "detectors.hpp"

using namespace follow_the_drow;

#define FREQUENCY_INIT 5
#define FREQUENCY_MAX 25

#define UNCERTAINTY_MIN 1
#define UNCERTAINTY_MAX 1
#define UNCERTAINTY_INC 0.05

#define CLUSTER_THRESHOLD 0.1

#define LEG_SIZE_MIN 0.05
#define LEG_SIZE_MAX 0.25
#define LEGS_DISTANCE_MIN 0.0
#define LEGS_DISTANCE_MAX 0.7

#define CHEST_SIZE_MIN 0.3
#define CHEST_SIZE_MAX 0.8

#define DISTANCE_LEVEL 0.6

#if defined ROS_ENVIRONMENT
#include "ros/ros.h"
#define LOG(...) ROS_INFO(__VA_ARGS__)
#else
#include <cstdio>
#define LOG(...) printf(__VA_ARGS__)
#endif


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
            float twoLegsDistance = legClusters[loopLeftLeg].distanceTo(legClusters[loopRightLeg]);
            bool personLegsDetected = (twoLegsDistance > LEGS_DISTANCE_MIN && twoLegsDistance < LEGS_DISTANCE_MAX);
            if (!personLegsDetected) continue;

            if (!topScanReceived) {
                detection.push_back(legClusters[loopLeftLeg].middleBetween(legClusters[loopRightLeg]));
                continue;
            }

            for (int loopChest = 0; loopChest < chestClusters.size(); loopChest++) {
                float leftChestDistance = legClusters[loopLeftLeg].distanceTo(chestClusters[loopChest]);
                float rightChestDistance = legClusters[loopRightLeg].distanceTo(chestClusters[loopChest]);
                bool personChestDetected = (leftChestDistance < DISTANCE_LEVEL) && (rightChestDistance < DISTANCE_LEVEL);
                if (!personChestDetected) continue;

                detection.push_back(chestClusters[loopChest].betweenPoint());
            }
        }
    if (logging && detection.size() > 0) LOG("%ld persons have been detected\n", detection.size());
    return detection;
}

const std::vector<Tracked> StatelessDetector::estimatePreviousPeople(const Point& newOdometry) {
    std::vector<Tracked> tracked(previousPeople.size());
    for (int loop = 0; loop < previousPeople.size(); loop++) {
        Tracked previous = previousPeople[loop];
        float angle = newOdometry.z - previousPosition.z;
        float sinus = sin(angle);
        float cosinus = cos(angle);
        float xRot = previous.x * cosinus - previous.y * sinus;
        float yRot = previous.x * sinus + previous.y * cosinus;
        float xTra = xRot + (newOdometry.x - previousPosition.x);
        float yTra = yRot + (newOdometry.y - previousPosition.y);
        tracked[loop] = Tracked(xTra, yTra, previous.z, previous.frequency, previous.uncertainty);
    }
    return tracked;
}

const std::vector<Tracked> StatelessDetector::pullAssociatedPeople(std::vector<Point>& detectedPeople, const std::vector<Tracked>& estimatedPrevious) {
    std::vector<Tracked> associated;
    for (int prevLoop = 0; prevLoop < estimatedPrevious.size(); prevLoop++) {
        int associatedIndex = -1;
        float minDistance = UNCERTAINTY_MAX;
        for (int detLoop = 0; detLoop < detectedPeople.size(); detLoop++) {
            float distance = estimatedPrevious[prevLoop].distanceTo(detectedPeople[detLoop]);
            if (distance < minDistance) {
                associatedIndex = detLoop;
                minDistance = distance;
            }
        }
        if (associatedIndex != -1) {
            associated.push_back(Tracked(detectedPeople[associatedIndex], FREQUENCY_INIT, FREQUENCY_MAX));
            detectedPeople.erase(detectedPeople.begin() + associatedIndex);
        } else {
            float uncertainty = estimatedPrevious[prevLoop].uncertainty + UNCERTAINTY_INC;
            int frequency = estimatedPrevious[prevLoop].frequency + 1;
            if (frequency > FREQUENCY_MAX) associated.push_back(Tracked(estimatedPrevious[prevLoop], frequency, uncertainty));
        }
    }
    return associated;
}

const std::vector<Tracked> StatelessDetector::trackDetectedPeople(const std::vector<Point>& detectedPeople) {
    std::vector<Tracked> tracked = previousPeople;
    for (int loop = 0; loop < detectedPeople.size(); loop++) tracked.push_back(Tracked(detectedPeople[loop], FREQUENCY_INIT, FREQUENCY_MAX));
    return tracked;
}

const std::vector<Point> StatelessDetector::getCurrentlyDetectedPeople() {
    std::vector<Point> current;
    for (int loop = 0; loop < previousPeople.size(); loop++)
        if (previousPeople[loop].frequency == FREQUENCY_INIT)
            current.push_back(previousPeople[loop]);
    return current;
}

const std::vector<Point> StatelessDetector::forward(const std::vector<Point>& latestBottomScan, const std::vector<Point>& latestTopScan, bool topScanReceived, const Point& odometry, bool odometryReceived) {
    std::vector<Cluster> legs = detectLegs(performClustering(latestBottomScan));
    std::vector<Cluster> chests;
    if (topScanReceived) chests = detectChests(performClustering(latestTopScan));

    std::vector<Point> detected = detectPeople(legs, chests, topScanReceived);
    if (previousPeople.size() > 0) {
        std::vector<Tracked> estimatedPrevious = previousPeople;
        if (odometryReceived) estimatedPrevious = estimatePreviousPeople(odometry);
        previousPeople = pullAssociatedPeople(detected, estimatedPrevious);
    }

    previousPeople = trackDetectedPeople(detected);
    return getCurrentlyDetectedPeople();
}

StatelessDetector::StatelessDetector(bool verbose): AbstractDetector(verbose) {}
