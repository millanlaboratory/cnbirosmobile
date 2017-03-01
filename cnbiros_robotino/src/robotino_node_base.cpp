#include <ros/ros.h>
#include "RobotinoBase.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	float frequency;
	std::string hostname, topic, odometry_topic;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_base");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create robotino instances
	RobotinoBase* robotino;
	robotino = new RobotinoBase("192.168.1.3", 10.0f);  // <-- TO BE CHANGED
	robotino->Register(&node);							// <-- TO BE CHANGED

	// Subscribe to required topic
	robotino->Subscribe("/cmd_vel");							// <-- TO BE CHANGED
	//robotino->AdvertiseOdometry("/base_odometry");		// <-- TO BE CHANGED

	// Run main loop
	robotino->Run();
		
	ros::spin();
		
	delete robotino;

	return 0;
}
