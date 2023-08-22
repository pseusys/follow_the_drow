#include <stdexcept>
#include <algorithm>

#include "follow_the_drow/misc.hpp"

#include "tracker.hpp"


const std::string getStringFromTrackerPolicy(const TrackerPolicy policy) {
    switch (policy) {
        case TrackerPolicy::First: return "first";
        case TrackerPolicy::Tracked: return "tracked";
        case TrackerPolicy::Closest: return "closest";
        default: throw std::runtime_error("Bad tracker policy value!");
    }
}

const TrackerPolicy getTrackerPolicyFromString(const std::string string) {
    std::string lower = lowercase(string);
    if (lower == "first") return TrackerPolicy::First;
    if (lower == "tracked") return TrackerPolicy::Tracked;
    if (lower == "closest") return TrackerPolicy::Closest;
    throw std::runtime_error("Bad tracker policy name '" + string + "'!");
}


const follow_the_drow::Point trackFirst(std::vector<follow_the_drow::Point>& points) {
    return points[0];
}

const follow_the_drow::Point trackTracked(std::vector<follow_the_drow::Point>& points, const follow_the_drow::Point previous) {
    double minDist = previous.distanceTo(points[0]);
    follow_the_drow::Point tracked = points[0];
    for (follow_the_drow::Point point: points) if (previous.distanceTo(point) < minDist) {
        minDist = previous.distanceTo(point);
        tracked = point;
    }
    return tracked;
}

const follow_the_drow::Point trackClosest(std::vector<follow_the_drow::Point>& points) {
    return trackTracked(points, follow_the_drow::Point(0, 0));
}

const follow_the_drow::Point track(const TrackerPolicy policy, std::vector<follow_the_drow::Point>& points) {
    return track(policy, follow_the_drow::Point(0, 0), points);
}

const follow_the_drow::Point track(const TrackerPolicy policy, const follow_the_drow::Point previous, std::vector<follow_the_drow::Point>& points) {
    if (points.size() < 1) throw std::runtime_error("Points vector is empty!");
    switch (policy) {
        case TrackerPolicy::First: return trackFirst(points);
        case TrackerPolicy::Tracked: return trackTracked(points, previous);
        case TrackerPolicy::Closest: return trackClosest(points);
        default: throw std::runtime_error("Bad tracker policy value!");
    }
}
