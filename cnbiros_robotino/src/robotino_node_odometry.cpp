#include <ros/ros.h>
#include "cnbiros_robotino/Odometry.hpp"

int main(int argc, char** argv) {

	std::string hostname = "192.168.1.3";

	// ROS initialization
	ros::init(argc, argv, "robotino_node_odometry");
	
	// Node initialization
	ros::NodeHandle node;
	cnbiros::robotino::Odometry* odometry;

	// Get parameters from the server
	hostname = node.param("hostname", hostname);
	
	// Create motor instance
	odometry = new cnbiros::robotino::Odometry(hostname);

	// Run main loop
	odometry->Run();
		
	delete odometry;

	return 0;
}
