#include <ros/ros.h>

#include "cnbiros_bci/Control.hpp"

int main(int argc, char** argv) {


	// ROS initialization
	ros::init(argc, argv, "bci_control_discrete");

	// Node initialization
	cnbiros::bci::Control control(cnbiros::bci::Control::AsDiscrete, "bci_discrete");
	
	control.AddSource("/tidproxy");

	control.SetFilterId("/dev");
	// Run the TiD proxy 
	control.Run();

	// Spin last time before exit
	ros::spin();

	return EXIT_SUCCESS;
}
