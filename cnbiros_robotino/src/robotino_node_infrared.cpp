#include <ros/ros.h>

#include "RobotinoInfrared.hpp"

#define ROBOTINO_HOSTNAME_IP 				"192.168.1.3"			// Default IP address for robotino base
#define ROBOTINO_TOPIC_INFRARED 			"/robotino_velocity"	// Default topic name to be written
#define ROBOTINO_INFRARED_UPDATE_FREQUENCY 	10 						// Default update loop rate [Hz]
#define ROBOTINO_INFRARED_DECAY 			1 						// Default decay time [seconds]

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	std::string hostname 	  = ROBOTINO_HOSTNAME_IP;
	std::string TopicInfrared = ROBOTINO_TOPIC_INFRARED;
	int frequency 		  	  = ROBOTINO_INFRARED_UPDATE_FREQUENCY;
	float decay 			  = ROBOTINO_INFRARED_DECAY;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_infrared");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Retrieve information from launcher
	ros::param::get("/robotino_node_main/hostname", hostname);
	ros::param::get("/robotino_node_infrared/topicname", TopicInfrared);
	ros::param::get("/robotino_node_infrared/frequency", frequency);
	ros::param::get("/robotino_node_infrared/decay", decay);

	// Create sensors instances
	RobotinoInfrared* infrared;
	infrared = new RobotinoInfrared(hostname, frequency);
	infrared->Register(&node);
	infrared->SetDecayTime(decay);

	// Advertise/Subscribe to all required topics
	infrared->Advertise(TopicInfrared);

	// Run loop
	infrared->Process();
		
	ros::spin();
		
	delete infrared;

	return 0;
}
