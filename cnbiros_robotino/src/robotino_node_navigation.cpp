#include <ros/ros.h>
#include "ForceField.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_navigation");
	
	// Create fusion object
	ForceField* navigation;	
	navigation = new ForceField;
	
	// Register to the node and advertise/subscribe to topics
	navigation->AddSource("/fusion");
	//controller->SetSubscriber("/odom", Navigation::MsgType::AsOdometry);

	// Setting up all parameters
	navigation->SetGridLayer("fusion");

	// Run main loop
	printf("Navigation -> Run()\n");
	navigation->Run();


	ros::shutdown();
	delete navigation;

	return 0;
}

