#include <ros/ros.h>

#include "CnbiInterface.hpp"
#include "TiCProxy.hpp"

#define CNBIROS_BCI_TIC_READING		"/ctrl10"
#define CNBIROS_BCI_TIC_WRITING 	"/ctrl11"
#define CNBIROS_BCI_TIC_SUBTOPICS	"/tic_to_cnbiloop"

int main(int argc, char** argv) {

	std::string 			 claddress  {""};							// CnbiLoop address
	std::vector<std::string> rpipes	   	{CNBIROS_BCI_TIC_READING}; 		// Reading pipes
	std::vector<std::string> wpipes	   	{CNBIROS_BCI_TIC_WRITING};  	// Writing pipes
	std::vector<std::string> stopics	{CNBIROS_BCI_TIC_SUBTOPICS};	// Subtopics

	// ROS initialization
	ros::init(argc, argv, "tobi_interface_c");

	// Node initialization
	ros::NodeHandle node("~");
	cnbiros::bci::CnbiInterface* 	cnbiloop;
	cnbiros::bci::TiCProxy* 		proxy;
	
	// Get parameters from server (with default values)
	node.param("cnbiloop", 	claddress, 	claddress);
	node.param("frompipes", rpipes, 	rpipes);
	node.param("topipes", 	wpipes, 	wpipes);
	node.param("subtopics", stopics, 	stopics);

	// Check the size of the writing pipes and the subtopics
	if(wpipes.size() != stopics.size()) {
		ROS_FATAL("%s reports an error during initialization: number of writing pipes"
				  "and subscribe topics is different. Shutting down the node", 
				  node.getNamespace().c_str()); 
		node.shutdown();
		exit(EXIT_FAILURE);
	}

	// Instanciate the cnbiloop interface
	cnbiloop = new cnbiros::bci::CnbiInterface(claddress);

	// Instanciate the TiC proxy
	proxy = new cnbiros::bci::TiCProxy();

	// Attach to all the provided reading pipes
	for(auto itr = rpipes.begin(); itr != rpipes.end(); ++itr) {
		if(proxy->Attach(cnbiros::bci::TiCProxy::AsReader, *itr) == true) {
			ROS_INFO("%s attached to pipe \'%s\' as reader", proxy->GetName().c_str(), (*itr).c_str());
		} else {
			ROS_WARN("%s cannot attach to pipe \'%s\' as reader", proxy->GetName().c_str(), (*itr).c_str());
		}
	}

	// Attach to all the provided writing pipes
	for(auto itw = wpipes.begin(), itt = stopics.begin(); itw != wpipes.end(); ++itw, ++itt) {
		if(proxy->Attach(cnbiros::bci::TiCProxy::AsWriter, *itw, *itt) == true) {
			ROS_INFO("%s attached to pipe \'%s\' as writer and subscribed on topic \'%s\'", 
					 proxy->GetName().c_str(), (*itw).c_str(), (*itt).c_str());
		} else {
			ROS_WARN("%s cannot attach to pipe \'%s\' as writer", 
					  proxy->GetName().c_str(), (*itw).c_str());
		}
	}

	// Run the TiC proxy 
	proxy->Run();

	// Spin last time before exit
	ros::spin();

	node.shutdown();
	return EXIT_SUCCESS;
}
