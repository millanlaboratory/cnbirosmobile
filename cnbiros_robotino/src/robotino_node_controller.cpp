#include <ros/ros.h>
#include "ForceField.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_navigation");
	
	// Create node handler
	ros::NodeHandle node("~");
	
	// Create fusion object
	ForceField* controller;	
	controller = new ForceField(&node);
	
	// Register to the node and advertise/subscribe to topics
	controller->SetSubscriber("/fusion", Navigation::MsgType::AsGridMap);
	controller->SetSubscriber("/odom", Navigation::MsgType::AsOdometry);
	controller->SetPublisher("/cmd_vel");

	// Setting up all parameters
	controller->SetGridLayer("fusion");
	controller->SetInfluence(1.0f);
	controller->SetObstruction(0.4f);
	controller->SetStrength(0.5f);

	// Run main loop
	controller->Run();
	
	delete controller;

	return 0;
}

