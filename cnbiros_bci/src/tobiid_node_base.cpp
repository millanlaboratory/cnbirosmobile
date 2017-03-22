#include <ros/ros.h>
#include <cnbiloop/ClLoop.hpp>
#include <cnbiloop/ClTobiId.hpp>

#include "TiDProxy.hpp"

using namespace cnbiros::bci;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "tobiid_node_base");
	
	// CNBI loop configuration
	CcCore::OpenLogger("rostobiid");
	
	ROS_INFO("Connecting to loop...");
	ClLoop::Configure();
	if(ClLoop::Connect() == false) {
		ROS_FATAL("Cannot connect to loop. Exit");
		ros::shutdown();
	}
	ROS_INFO("Loop connected");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create interface instance
	TiDProxy* id;
	id = new TobiId(&node, ClTobiId::GetOnly);

	// Initialization of interface
	id->SetFrequency(20.0f);

	// Attach interface
	if(id->Attach("/bus") == false)	{ 
		ros::shutdown();
	}


	// Run loop
	id->Run();
		
	ros::spin();
	
	CcCore::Exit(0);
		
	delete id;

	return 0;
}
