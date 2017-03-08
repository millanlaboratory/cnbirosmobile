#include <ros/ros.h>

#include "BciInput.hpp"

using namespace cnbiros::bci;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "bci_node_base");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create sensor instance
	BciInput* bci;
	bci = new BciInput(&node);

	// Initialization of sensor
	bci->SetFrequency(20.0f);
	bci->AdvertiseOn("/input_bci");


	// Run loop
	bci->Run();
		
	ros::spin();
		
	delete bci;

	return 0;
}
