#ifndef CNBIROS_ROBOTINO_MOTOR_HPP
#define CNBIROS_ROBOTINO_MOTOR_HPP

#include <rec/robotino/api2/OmniDrive.h>
#include <rec/robotino/api2/Odometry.h>

#include "cnbiros_core/Motor.hpp"
#include "cnbiros_robotino/Communication.hpp"

namespace cnbiros {
	namespace robotino {

//* Robotino Motor class
/**
 * \brief Class to handle motors of robotino
 *
 * This class handles the motors of robotino. It derives from the template class
 * cnbiros::core::Motor and it manages the OmniDrive class provided by the
 * rec::robotino::api2 library.
 *
 * \par General behavior:
 * As Motor class it subscribe on the topic "/cmd_vel". Every time a new
 * geometry_msgs::Twist message arrives, it set the velocity of the robotino
 * motors accordingly.
 * 
 * \par Usage:
 * The class can be used via the API provided by this library. However, a
 * generic implementation of a ROS node is already provided, and most of the
 * parameters  can be set via xml launch file. For more details, please refer to
 * robotino_node_motor.cpp executable.
 *
 * \sa robotino_node_motor.cpp
 */ 
class Motor : public core::Motor {
	public:	

		/*! \brief Constructor
		 * By default the name of the class is "motor". 
		 * \param name Name of the interface [default: "motor"]
		 */
		Motor(std::string name = "motor");
		
		/*! \brief Destructor
		 */
		 ~Motor(void);
		
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
	
		/*! \brief Set motor velocity
		 *
		 * This method sets the motor velocity according to the
		 * geometry_msgs::Twist message provided. In particular it sets the
		 * linear velocity along x (twist.linear.x), along y (twist.linear.y)
		 * and the angular velocity (twist.angular.z).
		 *
		 * \param twist 	Twist message with the velocity to be set
		 */
		void SetVelocity(const geometry_msgs::Twist& twist);
	private:
		void onRunning(void);

	private:
		Communication* com_;
		rec::robotino::api2::OmniDrive omnidrive_;
};

	}
}

#endif
