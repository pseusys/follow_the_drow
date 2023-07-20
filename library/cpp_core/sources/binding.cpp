#include <iostream>

#include <pybind11/pybind11.h>

#include "binding.hpp"

#define STRINGIFY(x) #x

using namespace follow_the_drow;


PythonDetectorFactory::PythonDetectorFactory(const DetectorType& detector, double minAngle, double incAngle): DetectorFactory(detector, minAngle, incAngle, -1, -1) {}

const std::vector<Point> PythonDetectorFactory::to_point_vector(std::vector<double>& measures) {
    std::vector<Point> points(measures.size());
    double measureAngle = minimumAngle;
    for (int measure = 0; measure < measures.size(); measure++, measureAngle += incrementAngle) points[measure] = Point::polar_to_cartesian(measures[measure], measureAngle);
    return points;
}

double* PythonDetectorFactory::to_raw_doubles(std::vector<Point>& points) {
    double *dump = new double[points.size() * 2];
    for (size_t i = 0; i < points.size(); i++) {
        dump[i * 2] = points[i].x;
        dump[i * 2 + 1] = points[i].y;
    }
    return dump;
}

const py::array_t<double> PythonDetectorFactory::forward(const py::array_t<double>& latestBottomScan, const py::array_t<double>& latestTopScan) {
    std::cout << "start!!" << std::endl;
    std::vector<double> bottomScan(latestBottomScan.data(), latestBottomScan.data() + latestBottomScan.size());
    std::cout << "bottom!!" << std::endl;
    std::vector<double> topScan(latestTopScan.data(), latestTopScan.data() + latestTopScan.size());
    std::cout << "top!!" << std::endl;
    std::vector<Point> result; // TODO: fix!!
    //std::vector<Point> result = detector->forward(PythonDetectorFactory::to_point_vector(bottomScan), PythonDetectorFactory::to_point_vector(topScan));
    double* raw_dump = &topScan.front(); // PythonDetectorFactory::to_raw_doubles(result);
    std::cout << "raw!!" << std::endl;
    return py::array_t<double>(std::vector<int>{topScan.size(), 1}, std::vector<int>{sizeof(double), sizeof(double)}, raw_dump);
    //return py::array_t<double>(std::vector<int>{result.size(), 2}, std::vector<int>{2 * sizeof(double), sizeof(double)}, raw_dump);
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
        .def(py::init<const DetectorType&, double, double>())
        .def("forward", &PythonDetectorFactory::forward, py::return_value_policy::take_ownership)
        .def("__repr__",
            [](const PythonDetectorFactory& factory) {
                return "<cpp_binding.DetectorFactory bound to '" + factory.typeName() + "'>";
            }
        );

#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
