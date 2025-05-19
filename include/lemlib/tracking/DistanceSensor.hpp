#pragma once

#include "hardware/Encoder/lidar.hpp"
//#include "hardware/Encoder/Encoder.hpp"
#include "hardware/Encoder/Sensor.hpp"
#include "hardware/Port.hpp"
#include "hardware/IMU/IMU.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "units/Pose.hpp"
#include <vector>

namespace lemlib {
/**
 * @brief class representing a tracking wheel
 *
 */
class DistanceSensor {
    public:
        
        
        DistanceSensor(Sensor* sensor, SmartPort port, Length offset);
        /**
         * @brief Construct a new Tracking Wheel object using an Optical Shaft Encoder
         *
         * @param port the port of the distance sensor
    
         * @param offset how far the distance sensor is from the center line
         * 
         *
         * @b Example:
         * @code {.cpp}
         * // create a tracking wheel with an optical shaft encoder on ports 'A' and 'B',
         * // which is not reversed, has a diameter of 2 inches, and is offset 4 inches
         * // to the right, and has a gear ratio of 2:1
         * lemlib::TrackingWheel trackingWheel('A', 'B', false, 2_in, 4_in, 2);
         * @endcode
         */
        Length getDistance();
        Length getOffset();

        /**
         * @brief Get the distance of the sensor
         *
         * Since the internal encoder object is abstract, it's not known what values errno may be
         * set to in case of a failure.
         *
         * @return INFINITY an error has occurred, possibly setting errno
         * @return Length the distance the tracking wheel has traveled since the last time
         * the function was called
         *
         * @b Example:
         * @code {.cpp}
         * void autonomous() {
         *   // create tracking wheel
         *   lemlib::V5RotationSensor encoder(3, true);
         *   lemlib::TrackingWheel trackingWheel(&encoder, 2.75_in, -3_in);
         *
         *   // loop forever
         *   while (true) {
         *     std::cout << "distance traveled: "
         *               << trackingWheel.getDistanceTraveled()
         *               << std::endl;
         *     pros::delay(10);
         *   }
         * }
         * @endcode
         */
        
        int reset();
        /**
         * @brief Destroy the Tracking Wheel object
         *
         * This function deallocates the encoder pointer unless the pointer was passed to the constructor
         */
        ~DistanceSensor();
    private:
        Sensor* m_sens;
        SmartPort m_port;
        //Length m_diameter;
        Length m_offset;
        //Number m_ratio;
        //Length m_lastTotal;
        bool m_deallocate = false;
};


} // namespace lemlib
