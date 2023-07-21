#pragma once

#include "geometry_msgs/Point.h"


geometry_msgs::Point cartesian_to_polar(const geometry_msgs::Point point);
geometry_msgs::Point polar_to_cartesian(const geometry_msgs::Point point);

geometry_msgs::Point bottom_laser_transform_translation();
geometry_msgs::Point bottom_laser_transform_rotation();
geometry_msgs::Point top_laser_transform_translation();
geometry_msgs::Point top_laser_transform_rotation();
