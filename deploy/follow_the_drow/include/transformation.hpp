#ifndef DETECTION_TRANSFORMATOR_HPP
#define DETECTION_TRANSFORMATOR_HPP

#include "geometry_msgs/Point.h"

geometry_msgs::Point cartesian_to_polar(geometry_msgs::Point point);
geometry_msgs::Point polar_to_cartesian(geometry_msgs::Point point);

geometry_msgs::Point bottom_laser_transform_translation();
geometry_msgs::Point bottom_laser_transform_rotation();
geometry_msgs::Point top_laser_transform_translation();
geometry_msgs::Point top_laser_transform_rotation();

float cartesian_distance(geometry_msgs::Point first, geometry_msgs::Point second);

#endif // DETECTION_TRANSFORMATOR_HPP
