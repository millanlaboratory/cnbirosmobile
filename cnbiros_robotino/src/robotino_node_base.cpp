#include <ros/ros.h>
#include "RobotinoBase.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	// ROS initialization
	ros::init(argc, argv, "robotino_node_base");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create robotino instances
	RobotinoBase* robotino;
	robotino = new RobotinoBase("192.168.1.3", &node);

	// Run main loop
	robotino->Run();
		
	delete robotino;

	return 0;
}
