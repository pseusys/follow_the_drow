#include <pybind11/pybind11.h>

#include "binding.hpp"

#define STRINGIFY(x) #x

using namespace follow_the_drow;


PythonDetectorFactory::PythonDetectorFactory(const DetectorType& detector, const bool verbose): DetectorFactory(detector, verbose) {}

const std::vector<Point> PythonDetectorFactory::toPointVector(std::vector<double>& measures, double minAngle, double incAngle) {
    std::vector<Point> points(measures.size());
    double measureAngle = minAngle;
    for (int measure = 0; measure < measures.size(); measure++, measureAngle += incAngle) points[measure] = Point::polarToCartesian(measures[measure], measureAngle);
    return points;
}

double* PythonDetectorFactory::toRawDoubles(std::vector<Point>& points) {
    double *dump = new double[points.size() * 2];
    for (size_t i = 0; i < points.size(); i++) {
        dump[i * 2] = points[i].x;
        dump[i * 2 + 1] = points[i].y;
    }
    return dump;
}

const py::array_t<double> PythonDetectorFactory::forwardOne(const py::array_t<double>& latestBottomScan, double minAngle, double incAngle) {
    std::vector<double> bottomScan(latestBottomScan.data(), latestBottomScan.data() + latestBottomScan.size());
    std::vector<Point> result = detector->forward(PythonDetectorFactory::toPointVector(bottomScan, minAngle, incAngle));
    double* raw_dump = PythonDetectorFactory::toRawDoubles(result);
    return py::array_t<double>(std::vector<int>{result.size(), 2}, std::vector<int>{2 * sizeof(double), sizeof(double)}, raw_dump);
}

const py::array_t<double> PythonDetectorFactory::forwardBoth(const py::array_t<double>& latestBottomScan, const py::array_t<double>& latestTopScan, double minAngle, double incAngle) {
    std::vector<double> bottomScan(latestBottomScan.data(), latestBottomScan.data() + latestBottomScan.size());
    std::vector<double> topScan(latestTopScan.data(), latestTopScan.data() + latestTopScan.size());
    std::vector<Point> result = detector->forward(PythonDetectorFactory::toPointVector(bottomScan, minAngle, incAngle), PythonDetectorFactory::toPointVector(topScan, minAngle, incAngle));
    double* raw_dump = PythonDetectorFactory::toRawDoubles(result);
    return py::array_t<double>(std::vector<int>{result.size(), 2}, std::vector<int>{2 * sizeof(double), sizeof(double)}, raw_dump);
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
