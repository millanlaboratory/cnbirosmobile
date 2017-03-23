#include <ros/ros.h>

#include "CnbiInterface.hpp"

using namespace cnbiros::bci;

//! \todo To clean and review
int main(int argc, char** argv) {

	std::string address = "127.0.0.1:8123";

	// ROS initialization
	ros::init(argc, argv, "cnbiinterface_node_example");
	ros::NodeHandle node("~");
	ros::Rate rate(10);

	// Cnbi Loop initialization
	CnbiInterface interface(address);

	// Cnbi Loop connection (blocking)
	interface.Connect(true);




	while(node.ok()) {
		ros::spinOnce();
		rate.sleep();
	}


	return 0;
}
