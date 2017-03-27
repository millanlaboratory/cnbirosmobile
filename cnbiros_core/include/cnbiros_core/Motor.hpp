#ifndef CNBIROS_CORE_MOTOR_HPP
#define CNBIROS_CORE_MOTOR_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_services/SetTwist.h"
#include "cnbiros_services/Reset.h"

namespace cnbiros {
	namespace core {

//* Motor class
/**
 * \brief Motor class is the base class to interact with robot motors
 *
 * Motor() class represents the base class for different type of motors
 * developed in the cnbiros_mobile framework. 
 *
 * \par General description:
 * The basic behavior of the Motor() class is to read data on the subscribe
 * "/cmd_vel" topic and to set the velocity of the motors of the device. If a
 * different publishing topic is required, please consider ros remap tool
 * instead of changing manually the topic name. Based on RosInterface(), this
 * class implements some functionalities to help in controlling the motor
 * velocity.
 *
 * \par Basic methods and members:
 * - GetVelocity(), SetVelocity(): 
 * These two methods are used to get and set the velocity of the motors,
 * respectively. In particular, SetVelocity() is a pure abstract method specific
 * for each motor hardware and to be implemented in a derived class. 
 * - Reset():
 * This method resets the current velocity of the robot.
 * - onStop(), onStart():
 * Callbacks called when the class is required to stop/start.
 * - onRunning():
 * Main callback called when the class is running.
 *
 * \par Services:
 * By default, the Motor() class implements two service to set a given velocity
 * or to reset (to 0) the current velocity. The services are accessible via ROS
 * methods on <i>set_twist</i> and <i>reset_twist</i>, respectively.
 *
 *
 */ 
class Motor : public RosInterface {
	public:

		/*! \brief Constructor
		 *
		 * \param name 	Name of the motor
		 */
		Motor(std::string name);

		/*! \brief Destructor
		 */
		virtual ~Motor(void);

		/*! \brief Get the current velocity
		 * 
		 * \param[out] 	twist 	Reference to a message twist
		 */
		void GetVelocity(geometry_msgs::Twist& twist);

		/*! \brief Pure virtual function to set the velocity
		 * 
		 * \param twist 	Reference to the twist to be set
		 */
		virtual void SetVelocity(const geometry_msgs::Twist& twist) = 0;

		/*! \brief Reset the current velocity to 0
		 */
		void Reset(void);
	protected:
		/*! \brief Callback when a new twist message is received
		 *
		 * This callback is executed every time a new message is received on the
		 * subscribed topic ("/cmd_vel"). By default, the method SetVelocity()
		 * is called.
		 *
		 * \param msg 	Twist message with the new velocity
		 *
		 */
		virtual void onReceived(const geometry_msgs::Twist& msg);

		/*! \brief Callback when the interface stops
		 * 
		 * By default, the velocity is set to 0.
		 */
		virtual void onStop(void);
		
		/*! \brief Callback when the interface starts
		 * 
		 * By default, the velocity is set to 0.
		 */
		virtual void onStart(void);

		/*! \brief Empty virtual callback executed when the interface is running
		 */
		virtual void onRunning(void) {};

	private:
		bool on_service_set_(cnbiros_services::SetTwist::Request& req,
							 cnbiros_services::SetTwist::Response& res);
		bool on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res);
	protected:
		std::string 			topic_;
		geometry_msgs::Twist	motor_twist_;

	private:
		ros::ServiceServer 	srv_set_;
		ros::ServiceServer 	srv_reset_;

};

	}
}

#endif
