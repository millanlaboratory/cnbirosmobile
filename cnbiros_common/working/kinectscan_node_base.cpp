#include <ros/ros.h>

#include "KinectScan.hpp"

using namespace cnbiros::common;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "kinectscan_node_base");
	
	// Create sensor instance
	KinectScan* scan;
	scan = new KinectScan;

	// Run loop
	scan->Run();
		
	delete scan;

	return 0;
}
