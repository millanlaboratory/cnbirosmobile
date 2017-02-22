#include <ros/ros.h>
#include "Robotino.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	float frequency;
	std::string hostname, topic;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_base");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Retrieve configuration parameters
	if(ros::param::get("/frequency", frequency) == false)
		ROS_ERROR("Parameter frequency not found");
	if(ros::param::get("/hostname", hostname) == false)
		ROS_ERROR("Parameter hostname not found");
	if(ros::param::get("/robotino_node_base/topic", topic) == false)
		ROS_ERROR("Parameter topic not found");

	// Create robotino instances
	Robotino* robotino;
	robotino = new Robotino(hostname, frequency);
	robotino->Register(&node);

	// Subscribe to required topic
	robotino->Subscribe(topic);

	// Run main loop
	robotino->Run();
		
	ros::spin();
		
	delete robotino;

	return 0;
}
