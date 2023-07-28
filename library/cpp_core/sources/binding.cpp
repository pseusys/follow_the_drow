#include <pybind11/pybind11.h>

#include "binding.hpp"

#define STRINGIFY(x) #x

using namespace follow_the_drow;


PythonDetectorFactory::PythonDetectorFactory(const DetectorType& detector, const bool verbose): DetectorFactory(detector, verbose) {}

const std::vector<Point> PythonDetectorFactory::toPointVector(std::vector<float>& measures, float minAngle, float incAngle) {
    std::vector<Point> points(measures.size());
    float measureAngle = minAngle;
    for (int measure = 0; measure < measures.size(); measure++, measureAngle += incAngle) points[measure] = Point::polarToCartesian(measures[measure], measureAngle);
    return points;
}

float* PythonDetectorFactory::toRawFloats(std::vector<Point>& points) {
    float *dump = new float[points.size() * 2];
    for (size_t i = 0; i < points.size(); i++) {
        dump[i * 2] = points[i].x;
        dump[i * 2 + 1] = points[i].y;
    }
    return dump;
}

const py::array_t<float> PythonDetectorFactory::forwardOne(const py::array_t<float>& latestBottomScan, float minAngle, float incAngle) {
    std::vector<float> bottomScan(latestBottomScan.data(), latestBottomScan.data() + latestBottomScan.size());
    std::vector<Point> result = detector->forward(PythonDetectorFactory::toPointVector(bottomScan, minAngle, incAngle));
    float* raw_dump = PythonDetectorFactory::toRawFloats(result);
    return py::array_t<float>(std::vector<int>{result.size(), 2}, std::vector<int>{2 * sizeof(float), sizeof(float)}, raw_dump);
}

const py::array_t<float> PythonDetectorFactory::forwardBoth(const py::array_t<float>& latestBottomScan, const py::array_t<float>& latestTopScan, float minAngle, float incAngle) {
    std::vector<float> bottomScan(latestBottomScan.data(), latestBottomScan.data() + latestBottomScan.size());
    std::vector<float> topScan(latestTopScan.data(), latestTopScan.data() + latestTopScan.size());
    std::vector<Point> result = detector->forward(PythonDetectorFactory::toPointVector(bottomScan, minAngle, incAngle), PythonDetectorFactory::toPointVector(topScan, minAngle, incAngle));
    float* raw_dump = PythonDetectorFactory::toRawFloats(result);
    return py::array_t<float>(std::vector<int>{result.size(), 2}, std::vector<int>{2 * sizeof(float), sizeof(float)}, raw_dump);
}


PYBIND11_MODULE(cpp_binding, m) {
    m.doc() = R"pbdoc(
        Pybind11 binding plugin for different c++ detectors
        ---------------------------------------------------

        .. currentmodule:: cpp_binding
    )pbdoc";

    py::enum_<DetectorType>(m, "DetectorType")
        .value("Stateless", DetectorType::Stateless)
        .value("Stateful", DetectorType::Stateful)
        .export_values();

    py::class_<PythonDetectorFactory>(m, "DetectorFactory")
        .def(py::init<const DetectorType&, bool>())
        .def("forward_one", &PythonDetectorFactory::forwardOne, py::return_value_policy::take_ownership)
        .def("forward_both", &PythonDetectorFactory::forwardBoth, py::return_value_policy::take_ownership)
        .def("__repr__",
            [](const PythonDetectorFactory& factory) {
                return "<cpp_binding.DetectorFactory bound to '" + factory.typeName() + "' detector>";
            }
        );

#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
