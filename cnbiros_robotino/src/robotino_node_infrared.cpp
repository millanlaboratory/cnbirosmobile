#include <ros/ros.h>

#include "RobotinoInfrared.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_infrared");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create sensor instance
	RobotinoInfrared* infrared;
	infrared = new RobotinoInfrared("192.168.1.3", &node);
	
	// Initialization of sensor
	infrared->SetFrequency(10.0f);
	infrared->AdvertiseOn("/sensor_infrared");
	infrared->SetGrid("infrared", 1.0f, 1.0f, 0.05f);

	// Run loop
	infrared->Run();
		
	delete infrared;

	return 0;
}
