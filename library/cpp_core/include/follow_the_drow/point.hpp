#pragma once

#include <ostream>

namespace follow_the_drow {
    class Point {
        public:
            double x, y, z;

            Point();
            Point(const Point& point);
            Point(double x, double y);
            Point(double x, double y, double z);

            static Point polarToCartesian(double distance, double angle);
            static Point cartesianToPolar(double x, double y, double z);

            double distanceTo(const Point& point) const;
            Point middleBetween(const Point& point) const;

            friend std::ostream& operator<<(std::ostream& stream, const Point& point);
    };
}
