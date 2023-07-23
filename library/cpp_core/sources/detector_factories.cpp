#include <stdexcept>

#include "misc.hpp"

#include "detector_factories.hpp"

using namespace follow_the_drow;


const std::string follow_the_drow::getStringFromDetectorType(const DetectorType type) {
    switch (type) {
        case DetectorType::Stateless: return "Stateless";
        default: throw std::runtime_error("Bad DetectorType value!");
    }
}

const DetectorType follow_the_drow::getDetectorTypeFromString(const std::string string) {
    std::string lower = lowercase(string);
    if (lower == "stateless") return DetectorType::Stateless;
    throw std::runtime_error("Bad detector type name '" + string + "'!");
}

const std::unique_ptr<AbstractDetector> follow_the_drow::getDetectorFromDetectorType(const DetectorType type) {
    switch (type) {
        case DetectorType::Stateless: return std::unique_ptr<AbstractDetector>(new StatelessDetector());
        default: throw std::runtime_error("Bad DetectorType value!");
    }
}

DetectorFactory::DetectorFactory(const DetectorType& detector, double minAngle, double incAngle, double minRange, double maxRange): type(detector), minimumAngle(minAngle), incrementAngle(incAngle), minimumRange(minRange), maximumRange(maxRange), detector(getDetectorFromDetectorType(detector)) {}

const std::string DetectorFactory::typeName() const {
    return getStringFromDetectorType(this->type);
}
