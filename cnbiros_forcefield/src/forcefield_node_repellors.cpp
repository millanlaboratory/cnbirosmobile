#include <ros/ros.h>
#include "cnbiros_forcefield/Actors.hpp"

using namespace cnbiros::forcefield;

int main(int argc, char** argv) {

	std::vector<std::string>	topics;
	std::vector<int> 			types;
	float decay;

	// ROS initialization
	ros::init(argc, argv, "fusion_node_repellors");
	
	// Create fusion object
	Actors* repellors;
	repellors = new Actors(Actors::AsRepellor, "repellors");				
	
	// Get parameters from server (with default values)
	repellors->getParam("decay",  decay);
	repellors->getParam("topics", topics);
	repellors->getParam("types",  types);
	
	// Check the size of the writing pipes and the subtopics
	if(topics.size() != types.size()) {
		ROS_FATAL("%s reports an error during initialization: number of topics"
				  "and types is different. Shutting down the node", 
				  ros::this_node::getName().c_str()); 
		repellors->shutdown();
		exit(EXIT_FAILURE);
	}


	// Add all the provided sources
	std::vector<int>::iterator itp = types.begin();
	for(auto itt = topics.begin(); itt != topics.end(); ++itt, ++itp) {
		repellors->AddSource((*itt), (*itp));
		ROS_INFO("%s added source %s", repellors->GetName().c_str(), (*itt).c_str());
	}

	// Configure fusion grid
	repellors->SetDecayTime(decay);

	// Run main loop
	repellors->Run();

	delete repellors;

	return 0;
}
