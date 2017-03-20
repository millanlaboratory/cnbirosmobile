#include <ros/ros.h>

#include "RosInterface.hpp"
#include "CnbiInterface.hpp"
#include "TobiIc.hpp"
#include "cnbiros_bci/TobiIc.h"


using namespace cnbiros::bci;

int main(int argc, char** argv) {

	std::string address = "127.0.0.1:8123";

	// ROS initialization
	ros::init(argc, argv, "tobiic_node_example");
	ros::NodeHandle node("~");
	ros::Rate rate(10);

	// Cnbi Loop initialization
	CnbiInterface interface(address);

	// Cnbi Loop connection (blocking)
	interface.Connect(true);

	// TobiId initialization
	TobiIc tobiic;

	// Attach as GetOnly
	tobiic.Attach("/ctrl0");

	// Attach as SetOnly on /dev on topic /test
	tobiic.Attach("/ctrl1", "/test");


	tobiic.Run();

	return 0;
}
