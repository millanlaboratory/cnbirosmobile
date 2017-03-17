#include <ros/ros.h>
#include "Fusion.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_fusion");
	
	// Create fusion object
	Fusion* 	fusion;
	fusion = new Fusion;				

	// Configure fusion grid
	fusion->SetFrequency(10.0f);
	fusion->SetDecayTime(0.5f);

	fusion->AddSource("/sensor_infrared");
	fusion->AddSource("/sensor_kinectscan");
	fusion->AddSource("/sensor_sonar");
	fusion->AddSource("/input_bci");

	// Run main loop
	fusion->Run();

	delete fusion;

	return 0;
}
