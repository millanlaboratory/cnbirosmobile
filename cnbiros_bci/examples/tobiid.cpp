#include <ros/ros.h>

#include "CnbiInterface.hpp"
#include "TiDProxy.hpp"


using namespace cnbiros::bci;

int main(int argc, char** argv) {

	std::string address = "127.0.0.1:8123";

	// ROS initialization
	ros::init(argc, argv, "tobiid_node_example");
	ros::NodeHandle node("~");
	ros::Rate rate(10);

	// Cnbi Loop initialization
	CnbiInterface interface(address);

	// Cnbi Loop connection (blocking)
	interface.Connect(true);

	// TobiId initialization
	TiDProxy tobiid;

	// Attach as GetOnly
	tobiid.Attach("/bus");

	// Attach as SetOnly on /dev on topic /test
	tobiid.Attach("/dev", "/test");


	tobiid.Run();

	return 0;
}
