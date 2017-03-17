#include <ros/ros.h>
#include "ForceField.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "forcefield_node_base");
	
	// Create fusion object
	ForceField* navigation;	
	navigation = new ForceField;

	// Run main loop
	navigation->Run();

	delete navigation;

	return 0;
}

