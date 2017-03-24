#include <ros/ros.h>

#include "cnbiros_robotino/Infrared.hpp"

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_infrared");
	
	// Create sensor instance
	cnbiros::robotino::Infrared* infrared;
	infrared = new cnbiros::robotino::Infrared("192.168.1.3");
	
	// Run loop
	infrared->Run();
	
	delete infrared;

	return 0;
}
