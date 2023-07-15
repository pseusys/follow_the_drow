#include <stdexcept>
#include <iostream>

#include "detector_factories.hpp"


DetectorFactory::DetectorFactory(const DetectorType& detector, double minAngle, double incAngle, double minRange, double maxRange): type(detector), minimumAngle(minAngle), incrementAngle(incAngle), minimumRange(minRange), maximumRange(maxRange) {
    switch (detector) {
        case DetectorType::Stateless:
            this->detector = std::unique_ptr<AbstractDetector>(new StatelessDetector());
            break;
    
        default:
            throw std::runtime_error("Invalid detector type!");
    }
}

const std::string DetectorFactory::typeName() const {
    std::cout << "start!!" << std::endl;
    return typeid(*this->detector).name();
}
