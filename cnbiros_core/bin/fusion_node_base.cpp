#include <ros/ros.h>
#include "cnbiros_core/Fusion.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	std::vector<std::string> sources;
	float decay;

	// ROS initialization
	ros::init(argc, argv, "fusion_node_base");
	
	// Create fusion object
	Fusion* 	fusion;
	fusion = new Fusion;				

	fusion->AddSource("/camera/scan", 1);
	fusion->AddSource("/sensor_infrared", 2);
	fusion->AddSource("/ticproxy", 3);
	// Configure fusion grid
	fusion->SetDecayTime(0.5f);

	// Run main loop
	fusion->Run();

	delete fusion;

	return 0;
}
