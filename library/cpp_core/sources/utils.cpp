#include <cmath>

#include "utils.hpp"

using namespace follow_the_drow;


Point::Point(): Point(0, 0, 0) {}

Point::Point(double x, double y): Point(x, y, 0) {}

Point::Point(double x, double y, double z): x(x), y(y), z(z) {}

Point::Point(const Point& cluster): Point(this->x, this->y, this->z) {}

Point Point::polar_to_cartesian(double distance, double angle) {
    double x = distance * cos(angle);
    double y = distance * sin(angle);
    return Point(x, y, 0);
}

Point Point::cartesian_to_polar(double x, double y, double z) {
    double distance = sqrt(pow(x, 2) + pow(y, 2));
    double angle = x != 0 ? atan(y / x) : 0;
    return Point(distance, angle, z);
}

double Point::distance_to(const Point& point) const {
    return sqrt(pow((this->x - point.x), 2) + pow((this->y - point.y), 2));
}

Point Point::middle_between(const Point& point) const {
    double x = (this->x + point.x) / 2;
    double y = (this->y + point.y) / 2;
    double z = (this->z + point.z) / 2;
    return Point(x, y, z);
}