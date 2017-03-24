#include <ros/ros.h>
#include "cnbiros_robotino/Motor.hpp"

int main(int argc, char** argv) {

	std::string hostname = "192.168.1.3";

	// ROS initialization
	ros::init(argc, argv, "robotino_node_motor");
	
	// Node initialization
	ros::NodeHandle node;
	cnbiros::robotino::Motor* motor;

	// Get parameters from the server
	hostname = node.param("hostname", hostname);
	
	// Create motor instance
	motor = new cnbiros::robotino::Motor(hostname);

	// Run main loop
	motor->Run();
		
	delete motor;

	return 0;
}
