#include <ros/ros.h>

#include "cnbiros_robotino/Infrared.hpp"

int main(int argc, char** argv) {

	std::string hostname = "192.168.1.3";
	
	// ROS initialization
	ros::init(argc, argv, "robotino_node_infrared");
	
	// Node initialization
	ros::NodeHandle node;
	cnbiros::robotino::Infrared* infrared;
	
	// Get parameters from the server
	hostname = node.param("hostname", hostname);

	// Create sensor instance
	infrared = new cnbiros::robotino::Infrared(hostname);
	
	// Run loop
	infrared->Run();
	
	delete infrared;

	return 0;
}
