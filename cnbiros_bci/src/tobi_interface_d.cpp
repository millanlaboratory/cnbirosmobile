#include <ros/ros.h>

#include "CnbiInterface.hpp"
#include "TiDProxy.hpp"


int main(int argc, char** argv) {

	std::string 			 claddress  {""};								// CnbiLoop address
	std::vector<std::string> pipes	   	{"/bus", "/dev"}; 					// Reading/Writing pipes
	std::vector<std::string> stopics	{"/tid_ros2bus", "/tid_ros2dev"};	// Subtopics

	// ROS initialization
	ros::init(argc, argv, "tobi_interface_d");

	// Node initialization
	ros::NodeHandle node("~");
	cnbiros::bci::CnbiInterface* 	cnbiloop;
	cnbiros::bci::TiDProxy* 		proxy;
	
	// Get parameters from server (with default values)
	node.param("cnbiloop", 	claddress, 	claddress);
	node.param("pipes", 	pipes, 		pipes);
	node.param("subtopics", stopics, 	stopics);

	// Check the size of the writing pipes and the subtopics
	if(pipes.size() != stopics.size()) {
		ROS_FATAL("%s reports an error during initialization: number of pipes"
				  "and subscribe topics is different. Shutting down the node", 
				  node.getNamespace().c_str()); 
		node.shutdown();
		exit(EXIT_FAILURE);
	}

	// Instanciate the cnbiloop interface
	cnbiloop = new cnbiros::bci::CnbiInterface(claddress);

	// Instanciate the TiD proxy
	proxy = new cnbiros::bci::TiDProxy();


	// Attach to all the provided pipes as ReaderWriter
	for(auto itp = pipes.begin(), itt = stopics.begin(); itp != pipes.end(); ++itp, ++itt) {
		if(proxy->Attach(cnbiros::bci::TiDProxy::AsReaderWriter, *itp, *itt) == true) {
			ROS_INFO("%s attached to pipe \'%s\' as reader/writer and subscribed on topic \'%s\'", 
					 proxy->GetName().c_str(), (*itp).c_str(), (*itt).c_str());
		} else {
			ROS_WARN("%s cannot attach to pipe \'%s\' as reader/writer", 
					  proxy->GetName().c_str(), (*itp).c_str());
		}
	}

	// Run the TiD proxy 
	proxy->Run();

	// Spin last time before exit
	ros::spin();

	node.shutdown();
	return EXIT_SUCCESS;
}
