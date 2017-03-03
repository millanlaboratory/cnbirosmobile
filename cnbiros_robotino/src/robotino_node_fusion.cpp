#include <ros/ros.h>
#include "Fusion.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_fusion");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create fusion object
	Fusion* 	fusion;
	fusion = new Fusion(&node);				

	// Configure fusion grid
	fusion->SetFrequency(10.0f);
	fusion->AdvertiseOn("/fusion");
	fusion->SetGrid("fusion", 5.0f, 5.0f, 0.05f);
	fusion->SetDecayTime(1.0f);

	fusion->SubscribeTo("/sensor_infrared");
	fusion->SubscribeTo("/sensor_kinectscan");
	fusion->SubscribeTo("/sensor_sonar");

	// Run main loop
	fusion->Run();

	ros::spin();

	delete fusion;

	return 0;
}
