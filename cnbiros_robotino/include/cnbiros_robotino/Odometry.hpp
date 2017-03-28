#ifndef CNBIROS_ROBOTINO_ODOMETRY_HPP
#define CNBIROS_ROBOTINO_ODOMETRY_HPP

#include "tf/transform_datatypes.h"
#include <rec/robotino/api2/Odometry.h>
#include "cnbiros_core/Odometry.hpp"
#include "cnbiros_robotino/Communication.hpp"

namespace cnbiros {
	namespace robotino {

//* Robotino Odometry class
/**
 * \brief Class to handle odometry of robotino
 *
 * This class handles the odometry of robotino. It derives from the template
 * class cnbiros::core::Odometry and from the rec::robotino::api2::Odometry
 * class.
 *
 * \par General behavior:
 * As Odometry class it reads odometry from robotino and avery time there is new
 * data it publish on the topic "/odom".
 *
 * \par Usage:
 * The class can be used via the API provided by this library. However, a
 * generic implementation of a ROS node is already provided, and most of the
 * parameters  can be set via xml launch file. For more details, please refer to
 * robotino_node_odometry.cpp executable.
 *
 * \sa robotino_node_odometry.cpp
 *
 */ 
class Odometry : public cnbiros::core::Odometry, public rec::robotino::api2::Odometry {
	
	public:
		/*! \brief Constructor
		 * By default the name of the class is "odometry". 
		 * \param name Name of the interface [default: "odometry"]
		 */
		Odometry(std::string name = "odometry");

		/*! \brief Destructor
		 */
		~Odometry(void);
		
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

		/*! \brief Set robotino odometry
		 *
		 * This method sets the robotino odometry according to the
		 * nav_msgs::Odometry message provided. In particular it sets the
		 * odometry along x (pose.pose.position.x), along y
		 * (pose.pose.position.y) and the orientation (the yaw from the
		 * quaternion pose.orientation).
		 *
		 * \param odom 	Odometry message with the odometry to be set
		 */
		void SetOdometry(const nav_msgs::Odometry& odom);
	
	private:
		void readingsEvent(double x, double y, double omega, 
				           float vx, float vy, float vomega, 
						   unsigned int sequence);

		void onRunning(void);

	private:
		Communication* com_;


};

	}
}

#endif
