#include <ros/ros.h>
#include "RobotinoOdometry.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	float frequency;
	std::string hostname;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_odometry");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Create robotino instances
	RobotinoOdometry* odometry;
	odometry = new RobotinoOdometry("192.168.1.3", &node, 10.0f); 	// <-- TO BE CHANGED

	// Run main loop
	odometry->Run();
		
	ros::spin();
		
	delete odometry;

	return 0;
}
