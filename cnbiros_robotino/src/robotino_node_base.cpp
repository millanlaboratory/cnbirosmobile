#include <ros/ros.h>
#include "RobotinoMotors.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_base");
	
	// Create robotino instances
	RobotinoMotors* robotino;
	robotino = new RobotinoMotors("192.168.1.3");

	// Run main loop
	robotino->Run();
		
	delete robotino;

	return 0;
}
