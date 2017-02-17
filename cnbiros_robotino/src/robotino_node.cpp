#include <ros/ros.h>


#include "Robotino.hpp"
#include "RobotinoInfrared.hpp"

#define DEFAULT_HOSTNAME_IP 	"192.168.1.3"
#define DEFAULT_TOPIC_VELOCITY 	"/robotino_velocity"
#define DEFAULT_TOPIC_INFRARED 	"/robotino_infrared"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	std::string hostname 	  = DEFAULT_HOSTNAME_IP;
	std::string TopicVelocity = DEFAULT_TOPIC_VELOCITY;
	std::string TopicInfrared = DEFAULT_TOPIC_INFRARED;

	// ROS initialization
	ros::init(argc, argv, "robotino_node");
	
	// Create node handler
	ros::NodeHandle node("~");
	ros::Rate urate(10);

	// Retrieve information from launcher
	node.getParam("hostname", hostname);
	node.getParam("topic_velocity", TopicVelocity);
	node.getParam("topic_infrared", TopicInfrared);

	// Create robotino instances
	Robotino* robotino;
	robotino = new Robotino(hostname);

	// Create sensors instances
	RobotinoInfrared* infrared;
	infrared = new RobotinoInfrared(hostname);

	// Advertise/Subscribe to all required topics
	robotino->Subscribe(&node, TopicVelocity);
	infrared->Advertise(&node, TopicInfrared);


	while (node.ok()) {

		robotino->Run();
		infrared->Read();
		
		ros::spinOnce();
		urate.sleep();
	}
		
	delete robotino;
	delete infrared;

	return 0;
}
