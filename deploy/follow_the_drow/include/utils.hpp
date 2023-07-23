#pragma once

#include "geometry_msgs/Point.h"


geometry_msgs::Point cartesianToPolar(const geometry_msgs::Point point);
geometry_msgs::Point polarToCartesian(const geometry_msgs::Point point);
