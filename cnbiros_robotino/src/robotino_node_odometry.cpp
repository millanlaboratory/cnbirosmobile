#include <ros/ros.h>
#include "cnbiros_robotino/Odometry.hpp"

/*! 
 * \file
 *
 * \brief Robotino odometry node
 *
 * \par General description:
 * This executable provides an implementation of a working robotino odometry node.
 * It can be used as it is and it can be configured via xml launch file.
 *
 * \par Main behavior:
 * This node (namely 'robotino_node_odometry') instantiates a
 * cnbiros::robotino::Odometry() class, reading from the robotino motor encoders
 * (via Odometry class provided in rec::robotino::api2 libraries) and publishing
 * a nav_msgs::Odometry on the topic "/odom". The node blocks until a connection
 * to the robot can be established.
 * 
 * \par XML launcher:
 * It is possible to configure the following parameters in the XML launcher:
 * - \p ~hostname 	(\p str, default: 192.168.1.3) \n 
 * 	 The address of the robotino base
 *
 * \sa cnbiros::robotino::Odometry()
 *
 */ 
int main(int argc, char** argv) {

	std::string hostname;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_odometry");
	
	// Node initialization
	cnbiros::robotino::Odometry* odometry;
	
	// Create motor instance
	odometry = new cnbiros::robotino::Odometry();

	// Get parameters from the server
	odometry->getParam("hostname", hostname);

	// Connect to the robot
	odometry->Connect(hostname);

	// Run main loop
	odometry->Run();
		
	delete odometry;

	return 0;
}
