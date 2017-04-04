#include <ros/ros.h>
#include "cnbiros_forcefield/DiscreteControl.hpp"

using namespace cnbiros::forcefield;

int main(int argc, char** argv) {

	std::string topic;
	std::string	pipe;
	float 		radius;
	std::map<std::string, float> 	cmd_angles;

	// ROS initialization
	ros::init(argc, argv, "bci_discrete_control");

	// Create object
	DiscreteControl* control;
	control = new DiscreteControl();

	control->getParam("idsrc", 		topic);
	control->getParam("idpipe", 	pipe);
	control->getParam("idradius", 	radius);
	control->getParam("cmd_angles", cmd_angles);

	control->AddSource(topic);

	control->SetRadius(radius);

	if(pipe.empty() == false) {
		control->SetFilter(pipe);
	}

	for(auto it = cmd_angles.begin(); it != cmd_angles.end(); ++it) {
		control->SetCommand(it->first, it->second);
	}

	ROS_INFO("Running the 'bci_discrete_control' with source=%s and pipe=%s", topic.c_str(), pipe.c_str());
	control->Run();

	delete control;

	return 0;
}


