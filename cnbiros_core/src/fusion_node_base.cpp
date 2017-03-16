#include <ros/ros.h>
#include "Fusion.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	std::vector<std::string> sources;
	float decay;

	// ROS initialization
	ros::init(argc, argv, "fusion_node_base");
	
	// Create fusion object
	Fusion* 	fusion;
	fusion = new Fusion;				

	// Configure fusion grid
	fusion->SetFrequency(10.0f);

	// Run main loop
	fusion->Run();

	delete fusion;

	return 0;
}
