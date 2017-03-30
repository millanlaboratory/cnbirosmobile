#include <ros/ros.h>
#include "cnbiros_robotino/Infrared.hpp"

/*! 
 * \file
 *
 * \brief Robotino infrared node
 *
 * \par General description:
 * This executable provides an implementation of a working robotino infrared
 * node. It can be used as it is and it can be configured via xml launch file.
 *
 * \par Main behavior:
 * This node (namely 'robotino_node_infrared') instantiates a
 * cnbiros::robotino::Infrared() class, reading from the robotino infrared
 * sensors and publishing a PointCloud message on the topic "/sensor_infrared".
 * The node blocks until a connection to the robot can be established.
 * 
 * \par XML launcher:
 * It is possible to configure the following parameters in the XML launcher:
 * - \p ~hostname 	(\p str, default: 192.168.1.3) \n 
 * 	 The address of the robotino base
 *
 * \sa cnbiros::robotino::Infrared()
 *
 */ 
int main(int argc, char** argv) {

	std::string hostname;
	
	// ROS initialization
	ros::init(argc, argv, "robotino_node_infrared");
	
	// Node initialization
	cnbiros::robotino::Infrared* infrared;
	
	// Create sensor instance
	infrared = new cnbiros::robotino::Infrared();
	
	// Get parameters from the server
	infrared->getParam("hostname", hostname);

	// Connect to the robot
	infrared->Connect(hostname);

	// Run loop
	infrared->Run();
	
	delete infrared;

	return 0;
}
