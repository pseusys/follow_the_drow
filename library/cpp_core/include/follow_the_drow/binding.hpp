#pragma once

#include <pybind11/numpy.h>

#include "detector_factories.hpp"

namespace py = pybind11;


namespace follow_the_drow {
    class PythonDetectorFactory: public DetectorFactory {
        private:
            const std::vector<Point> toPointVector(std::vector<double>& measures);
            double* toRawDoubles(std::vector<Point>& points);

        public:
            PythonDetectorFactory(const DetectorType& detector, double minAngle, double incAngle);

            const py::array_t<double> forwardOne(const py::array_t<double>& latestBottomScan);
            const py::array_t<double> forwardBoth(const py::array_t<double>& latestBottomScan, const py::array_t<double>& latestTopScan);
    };
}
