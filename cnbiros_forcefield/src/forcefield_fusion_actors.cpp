#include <ros/ros.h>
#include "cnbiros_forcefield/Actors.hpp"

using namespace cnbiros::forcefield;

int main(int argc, char** argv) {

	std::vector<std::string>	topics;
	std::vector<int> 			types;
	float decay;

	// ROS initialization
	ros::init(argc, argv, "forcefield_fusion_actors");
	
	// Create fusion object
	Actors* actors;
	actors = new Actors("actors");				
	
	// Get parameters from server (with default values)
	actors->getParam("decay",  decay);
	actors->getParam("topics", topics);
	actors->getParam("types",  types);
	
	// Check the size of the writing pipes and the subtopics
	if(topics.size() != types.size()) {
		ROS_FATAL("%s reports an error during initialization: number of topics"
				  "and types is different. Shutting down the node", 
				  ros::this_node::getName().c_str()); 
		actors->shutdown();
		exit(EXIT_FAILURE);
	}


	// Add all the provided sources
	std::vector<int>::iterator itp = types.begin();
	for(auto itt = topics.begin(); itt != topics.end(); ++itt, ++itp) {
		actors->AddSource((*itt), (*itp));
		ROS_INFO("%s added source %s", actors->GetName().c_str(), (*itt).c_str());
	}

	// Configure fusion grid
	actors->SetDecayTime(decay);

	// Run main loop
	actors->Run();

	delete actors;

	return 0;
}
