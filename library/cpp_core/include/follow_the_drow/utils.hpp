#pragma once

namespace follow_the_drow {
    class Point {
        public:
            double x, y, z;

            Point();
            Point(const Point& cluster);
            Point(double x, double y);
            Point(double x, double y, double z);

            static Point polar_to_cartesian(double distance, double angle);
            static Point cartesian_to_polar(double x, double y, double z);

            double distance_to(const Point& point) const;
            Point middle_between(const Point& point) const;
    };
}
