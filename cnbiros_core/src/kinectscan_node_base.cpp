#include <ros/ros.h>

#include "KinectScan.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "kinectscan_node_base");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create sensor instance
	KinectScan* scan;
	scan = new KinectScan(&node);

	// Initialization of sensor
	scan->SetFrequency(10.0f);

	// Run loop
	scan->Run();
		
	delete scan;

	return 0;
}
