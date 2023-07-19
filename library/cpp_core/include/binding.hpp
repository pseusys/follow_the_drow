#pragma once

#include <pybind11/numpy.h>

#include "detector_factories.hpp"

namespace py = pybind11;


class PythonDetectorFactory: public DetectorFactory {
    private:
        const std::vector<Point> to_point_vector(std::vector<double>& measures);
        double* to_raw_doubles(std::vector<Point>& points);

    public:
        PythonDetectorFactory(const DetectorType& detector, double minAngle, double incAngle);

        const py::array_t<double> forward(const py::array_t<double>& latestBottomScan, const py::array_t<double>& latestTopScan);
};
