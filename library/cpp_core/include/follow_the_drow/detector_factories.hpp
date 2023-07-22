#pragma once

#include <memory>
#include <string>

#include "detectors.hpp"


namespace follow_the_drow {
    enum DetectorType {
        Stateless,
        Stateful
    };

    const std::string getStringFromDetectorType(const DetectorType type);
    const DetectorType getDetectorTypeFromString(const std::string& string);
    const std::unique_ptr<AbstractDetector> getDetectorFromDetectorType(const DetectorType type);


    class DetectorFactory {
        private:
            const DetectorType type;

        protected:
            double minimumAngle, incrementAngle, minimumRange, maximumRange;
            const std::unique_ptr<AbstractDetector> detector;

        public:
            DetectorFactory(const DetectorType& detector, double minAngle, double incAngle, double minRanges, double maxRange);

            const std::string typeName() const;
    };
}
