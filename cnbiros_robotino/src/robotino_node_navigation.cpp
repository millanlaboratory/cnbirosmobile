#include <ros/ros.h>
#include "ForceField.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_navigation");
	
	// Create node handler
	ros::NodeHandle node("~");
	
	// Create fusion object
	ForceField* navigation;	
	navigation = new ForceField(&node);
	
	// Register to the node and advertise/subscribe to topics
	navigation->SubscribeTo("/fusion");
	navigation->AdvertiseOn("/cmd_vel");
	//controller->SetSubscriber("/odom", Navigation::MsgType::AsOdometry);

	// Setting up all parameters
	navigation->SetGridLayer("fusion");
	navigation->SetInfluence(1.0f);
	navigation->SetObstruction(0.4f);
	navigation->SetStrength(0.5f);

	// Run main loop
	printf("Navigation -> Run()\n");
	navigation->Run();


	ros::shutdown();
	delete navigation;

	return 0;
}

