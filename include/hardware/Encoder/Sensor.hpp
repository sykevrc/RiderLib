#pragma once

#include "units/units.hpp"

namespace lemlib {
/**
 * @brief abstract class for encoders
 *
 * An encoder is a device that measures the angle of a rotating object. This class is abstract, and must be implemented
 * by a subclass.
 */
class Sensor {
    public:
        /**
         * @brief whether the encoder is connected
         *
         * @return 0 if its not connected
         * @return 1 if it is connected
         * @return INT_MAX if there is an error, setting errno
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     Encoder* encoder;
         *     const int result = encoder->isConnected();
         *     if (result == 1) {
         *         std::cout << "Encoder is connected!" << std::endl;
         *     } else if (result == 0) {
         *         std::cout << "Encoder is not connected!" << std::endl;
         *     } else {
         *         std::cout << "Error checking if encoder is connected!" << std::endl;
         *     }
         * }
         * @endcode
         */
        virtual int isConnected() = 0;
        /**
         * @brief Get the relative angle measured by the encoder
         *
         * The relative angle measured by the encoder is the angle of the encoder relative to the last time the encoder
         * was reset. As such, it is unbounded.
         *
         * @return Angle the relative angle measured by the encoder
         * @return INFINITY if there is an error, setting errno
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     Encoder* encoder;
         *     const Angle angle = encoder->getAngle();
         *     if (angle == INFINITY) {
         *         std::cout << "Error getting relative angle!" << std::endl;
         *     } else {
         *         std::cout << "Relative angle: " << angle.convert(deg) << std::endl;
         *     }
         * }
         * @endcode
         */
        virtual Length get() = 0;
        virtual ~Sensor() = default;
};
} // namespace lemlib