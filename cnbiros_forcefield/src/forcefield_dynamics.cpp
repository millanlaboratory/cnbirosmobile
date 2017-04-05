#include <ros/ros.h>
#include "cnbiros_forcefield/Dynamics.hpp"

using namespace cnbiros::forcefield;

int main(int argc, char** argv) {

	float 		obstruction  = 0.4f;
	float 		spatialdecay = 0.5f;
	std::string layer 		 = "actors";
	std::string source 		 = "/fusion_actors";

	// ROS initialization
	ros::init(argc, argv, "forcefield_dynamics");

	// Create object
	Dynamics* navigation;
	navigation = new Dynamics();

	navigation->getParam("source",  	  source);
	navigation->getParam("obstruction",  obstruction);
	navigation->getParam("spatialdecay", spatialdecay);
	navigation->getParam("layer", 		  layer);

	navigation->AddSource(source);


	ROS_INFO("Running the 'forcefield_dynamics'");
	navigation->Run();

	delete navigation;

	return 0;
}


