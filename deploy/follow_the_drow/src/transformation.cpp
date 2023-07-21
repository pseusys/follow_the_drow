/**
 * The pivot of the robot is considered to be on level of its bottom laser, in the center.
 * Apparently, for 
*/

#include <cmath>

#include "transformation.hpp"


geometry_msgs::Point cartesian_to_polar(const geometry_msgs::Point point) {
    geometry_msgs::Point result;
    result.x = sqrt(pow(point.x, 2) + pow(point.y, 2));
    result.y = point.x != 0 ? atan(point.y / point.x) : 0;
    result.z = point.z;
    return result;
}

geometry_msgs::Point polar_to_cartesian(const geometry_msgs::Point point) {
    geometry_msgs::Point result;
    result.x = point.x * cos(point.y);
    result.y = point.x * sin(point.y);
    result.z = point.z;
    return result;
}


geometry_msgs::Point bottom_laser_transform_translation() {
    geometry_msgs::Point transform;
    transform.x = 0.15;
    transform.y = 0.0;
    transform.z = 0.0;
    return cartesian_to_polar(transform);
}

geometry_msgs::Point bottom_laser_transform_rotation() {
    geometry_msgs::Point transform;
    transform.x = 0.0;
    transform.y = 0.0;
    transform.z = 0.0;
    return transform;
}

geometry_msgs::Point top_laser_transform_translation() {
    geometry_msgs::Point transform;
    transform.x = 0.0;
    transform.y = 0.0;
    transform.z = 1.2;
    return cartesian_to_polar(transform);
}

geometry_msgs::Point top_laser_transform_rotation() {
    geometry_msgs::Point transform;
    transform.x = 0.0;
    transform.y = 0.05;
    transform.z = 0.0;
    return transform;
}
