#include <ros/ros.h>
#include "cnbiros_robotino/Motor.hpp"

/*! 
 * \file
 *
 * \brief Robotino motor node
 *
 * \par General description:
 * This executable provides an implementation of a working robotino motor node.
 * It can be used as it is and it can be configured via xml launch file.
 *
 * \par Main behavior:
 * This node (namely 'robotino_node_motor') instantiates a
 * cnbiros::robotino::Motor() class, reading from the topic "/cmd_vel" and
 * setting accordingly the velocity of the motors of robotino (via OmniDrive
 * class provided in rec::robotino::api2 libraries). The node blocks until a
 * connection to the robot can be established.
 * 
 * \par XML launcher:
 * It is possible to configure the following parameters in the XML launcher:
 * - \p ~hostname 	(\p str, default: 192.168.1.3) \n 
 * 	 The address of the robotino base
 *
 * \sa cnbiros::robotino::Motor()
 *
 */ 
int main(int argc, char** argv) {

	std::string hostname = "192.168.1.3";

	// ROS initialization
	ros::init(argc, argv, "robotino_node_motor");
	
	// Node initialization
	cnbiros::robotino::Motor* motor;

	// Create motor instance
	motor = new cnbiros::robotino::Motor();
	
	// Get parameters from the server
	hostname = motor->param("hostname", hostname);
	
	// Connect to the robot
	motor->Connect(hostname);
	
	// Run main loop
	motor->Run();
		
	delete motor;

	return 0;
}
