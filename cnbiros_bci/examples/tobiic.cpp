#include <ros/ros.h>

#include "RosInterface.hpp"
#include "CnbiInterface.hpp"
#include "TiCProxy.hpp"
#include "cnbiros_bci/TiCMessage.h"


using namespace cnbiros::bci;

int main(int argc, char** argv) {

	std::string address = "127.0.0.1:8123";
	std::string rpipe   = "/ctrl0";
	std::string wpipe   = "/ctrl1";

	// ROS initialization
	ros::init(argc, argv, "tobiic_node_example");
	ros::NodeHandle node("~");
	ros::Rate rate(10);

	// Cnbi Loop initialization
	CnbiInterface interface(address);

	// Cnbi Loop connection (blocking)
	interface.Connect(true);

	// TiCProxy initialization
	TiCProxy tobiic;

	// Attach as Reader
	if(tobiic.Attach(TiCProxy::AsReader, rpipe))
		ROS_INFO("%s attached to %s as reader", tobiic.GetName().c_str(), rpipe.c_str());

	// Attach as SetOnly on /dev on topic /test
	tobiic.Attach(TiCProxy::AsWriter, wpipe, "/test");
		ROS_INFO("%s attached to %s as writer", tobiic.GetName().c_str(), wpipe.c_str());


	tobiic.Run();

	return 0;
}
