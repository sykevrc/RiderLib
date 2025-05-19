#pragma once

#include "hardware/Encoder/Sensor.hpp"
#include "hardware/Port.hpp"
#include "pros/rotation.hpp"
#include "pros/device.hpp"
#include "pros/distance.h"

namespace lemlib {
/**
 * @brief Encoder implementation for the V5 Rotation sensor
 *
 */
class lidar : public Sensor{
	/**
	 * \addtogroup cpp-distance
	 *  @{
	 */
	public:
	/**
	 * Creates a Distance Sensor object for the given port.
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as a Distance Sensor
	 *
	 * \param port
	 *        The V5 port number from 1-21
	 *
	 * \b Example
	 * \code
	 * #define DISTANCE_PORT 1
	 *
	 * void opcontrol() {
	 *   Distance distance(DISTANCE_PORT);
	 * }
	 * \endcode
	 */
	lidar(SmartPort m_port, Length offset);
    static DistanceSensor from_pros_rot(pros::Distance sensor);

	/**
	 * Get the currently measured distance from the sensor in mm
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Distance Sensor
	 *
	 * \return The distance value or PROS_ERR if the operation failed, setting
	 * errno. Will return 9999 if the sensor can not detect an object.
	 *
	 * \b Example
	 * \code
	 * #define DISTANCE_PORT 1
	 *
	 * void opcontrol() {
	  Distance distance(DISTANCE_PORT);
	 *   while (true) {
	 *     printf("Distance confidence: %d\n", distance.get());
	 *     delay(20);
	 *   }
	 * }
	 * \endcode
	 */
	Length get() override;
	int isConnected() override;

    private:
        int m_port;
		double offset;
};
} // namespace lemlib