#pragma once

#include <pybind11/numpy.h>

#include "detector_factory.hpp"

namespace py = pybind11;


namespace follow_the_drow {
    class PythonDetectorFactory: public DetectorFactory {
        private:
            float minAngle, incAngle;  // TODO: add this and odometry to constructor params!

            const std::vector<Point> toPointVector(std::vector<float>& measures, float minAngle, float incAngle);
            const Point toPoint(std::vector<float>& measures);
            float* toRawFloats(std::vector<Point>& points);

        public:
            PythonDetectorFactory(const DetectorType& detector, const bool verbose);

            const py::array_t<float> forwardOne(const py::array_t<float>& latestBottomScan, const py::array_t<float>& latestOdometry, float minAngle, float incAngle);
            const py::array_t<float> forwardBoth(const py::array_t<float>& latestBottomScan, const py::array_t<float>& latestTopScan, const py::array_t<float>& latestOdometry, float minAngle, float incAngle);
    };
}
