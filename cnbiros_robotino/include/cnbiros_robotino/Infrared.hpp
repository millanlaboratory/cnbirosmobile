#ifndef CNBIROS_ROBOTINO_INFRARED_HPP
#define CNBIROS_ROBOTINO_INFRARED_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <rec/robotino/api2/DistanceSensorArray.h>

#include "cnbiros_robotino/Communication.hpp"
#include "cnbiros_core/Sensor.hpp"

#define CNBIROS_ROBOTINO_BASE_RADIUS 			0.2f				// [m]
#define CNBIROS_ROBOTINO_INFRARED_HEIGHT 		0.03f				// [m]
#define CNBIROS_ROBOTINO_INFRARED_RANGE_MAX		0.41f 				// [m]
#define CNBIROS_ROBOTINO_INFRARED_ANGLE_INC		(40.0f*M_PI)/180.0f // [rad] 

namespace cnbiros {
	namespace robotino {

//* Robotino Infrared class
/**
 * \brief Class to handle infrared sensors of robotino
 *
 * This class handles the infrared sensors of robotino. It derives from the
 * template class cnbiros::core::Sensor and from the robotino api
 * DistanceSensorArray class. It is instanciated as a PointCloud sensor.
 *
 * \par General behavior:
 * As Sensor class it provides all the public methods of the
 * cnbiros::core::Sensor class. Every time a new infrared data is available (via
 * the rec::robotino::api2::DistanceSensorArray callback), this class convert
 * the data in a PointCloud message and publishes it on topic "/sensor_infrared"
 * (by default). Position of the infrared sensors as well as the radius of the
 * robot are pre-defined and fixed.
 * 
 * \par Usage:
 * The class can be used via the API provided by this library. However, a
 * generic implementation of a ROS node is already provided, and most of the
 * parameters  can be set via xml launch file. For more details, please refer to
 * robotino_node_infrared.cpp executable.
 *
 * \sa robotino_node_infrared.cpp
 *
 */ 
class Infrared : public core::Sensor<sensor_msgs::PointCloud>, 
				 public rec::robotino::api2::DistanceSensorArray {

	public:
		/*! \brief Constructor of the class
		 *
		 * By default the name of the class is "infrared". This means that the
		 * PointCloud message will be publish on the topic "/sensor_infrared"
		 * \param name Name of the interface [default: "infrared"]
		 */ 
		Infrared(std::string name = "infrared");

		/*! \brief Destructor
		 */
		virtual ~Infrared(void);

		/*! \brief Connects to the robot base
		 *
		 * This function wraps the Connect() method of the Communication()
		 * class. As additional functionalities, it provides a blocking
		 * parameter (enabled by default) to wait for connection.
		 *
		 * \param hostname 	IP address of the robotino base
		 * \param blocking 	If true, it blocks until a connection is established
		 * 					[default: true]
		 * \return 			True if the connection is established, false
		 * 					otherwise
		 */
		bool Connect(std::string hostname, bool blocking = true);

		/*! \brief Resets the PointCloud message
		 *
		 * This function resets the PointCloud message that will be published.
		 */
		void Reset(void);

	private:
		void onRunning(void);
		void distancesChangedEvent(const float* ranges, unsigned int size);

	private:
		Communication* com_;

};


	}
}




#endif
