#include <ros/ros.h>


#include "Robotino.hpp"

#define ROBOTINO_HOSTNAME_IP 			"192.168.1.3"			// Default IP address for robotino base
#define ROBOTINO_TOPIC_VELOCITY 		"/robotino_velocity" 	// Default topic name to be read
#define ROBOTINO_MAIN_UPDATE_FREQUENCY 	10 						// Default update loop rate [Hz]

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	std::string hostname 	  = ROBOTINO_HOSTNAME_IP;
	std::string TopicVelocity = ROBOTINO_TOPIC_VELOCITY;
	int rate 		  		  = ROBOTINO_MAIN_UPDATE_FREQUENCY;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_main");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Retrieve information from launcher
	ros::param::get("/robotino_node_main/hostname", hostname);
	ros::param::get("/robotino_node_main/topicname", TopicVelocity);
	ros::param::get("/robotino_node_main/rate", rate);

	// Create robotino instances
	Robotino* robotino;
	robotino = new Robotino(hostname, rate);
	robotino->Register(&node);

	// Advertise/Subscribe to all required topics
	robotino->Subscribe(TopicVelocity);

	// Run loop
	robotino->Run();
		
	ros::spin();
		
	delete robotino;

	return 0;
}
