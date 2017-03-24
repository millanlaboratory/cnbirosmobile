#include <ros/ros.h>
#include "cnbiros_robotino/Motor.hpp"

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_motor");
	
	// Create robotino instances
	cnbiros::robotino::Motor* motor;
	motor = new cnbiros::robotino::Motor("192.168.1.3");

	// Run main loop
	motor->Run();
		
	delete motor;

	return 0;
}
