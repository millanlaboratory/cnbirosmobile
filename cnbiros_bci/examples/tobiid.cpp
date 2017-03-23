#include <ros/ros.h>

#include "CnbiInterface.hpp"
#include "TiDProxy.hpp"


using namespace cnbiros::bci;

int main(int argc, char** argv) {

	std::string address = "127.0.0.1:8123";
	std::string rpipe   = "/bus";
	std::string wpipe   = "/dev";
	std::string stopic  = "/tid_to_cnbiloop";

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

	// Attach as Reader
	if(tobiid.Attach(TiDProxy::AsReader, rpipe)) {
		ROS_INFO("%s attached to %s as reader", tobiid.GetName().c_str(), rpipe.c_str());
	}
	
	// Attach as Writer
	if(tobiid.Attach(TiDProxy::AsWriter, wpipe, stopic)) {
		ROS_INFO("%s attached to %s as writer for %s", tobiid.GetName().c_str(), 
				  wpipe.c_str(), stopic.c_str());
	}
	
	tobiid.Run();

	return 0;
}
