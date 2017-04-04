#include <ros/ros.h>
#include "cnbiros_forcefield/ContinuousControl.hpp"

using namespace cnbiros::forcefield;

int main(int argc, char** argv) {

	std::string topic;
	std::string	pipe;
	std::string	name;
	std::string	label;
	float 		radius;

	// ROS initialization
	ros::init(argc, argv, "bci_continuous_control");

	// Create object
	ContinuousControl* control;
	control = new ContinuousControl();

	control->getParam("icsrc", topic);
	control->getParam("icpipe", 	pipe);
	control->getParam("icname", 	name);
	control->getParam("iclabel", 	label);
	control->getParam("icradius", radius);

	control->AddSource(topic);

	control->SetFilter(pipe, name, label);
	control->SetRadius(radius);

	ROS_INFO("Running the 'bci_continuous_control' with source=%s and pipe=%s", topic.c_str(), pipe.c_str());
	control->Run();

	delete control;

	return 0;
}


