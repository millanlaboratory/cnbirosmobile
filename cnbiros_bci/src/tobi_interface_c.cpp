#include <ros/ros.h>

#include "cnbiros_bci/CnbiInterface.hpp"
#include "cnbiros_bci/TiCProxy.hpp"

/*! 
 * \file
 *
 * \brief Tobi interface C proxy node
 *
 * \par General description:
 * This executable provides an implementation of a working TiC proxy node. It
 * can be used as it is and it can be configured via xml launch file.
 *
 * \par Main behavior:
 * This node (namely 'tobi_interface_c') connects to the CNBI Loop (waiting
 * until the connection succed), attached to the provided pipes (as reader and as
 * writer) and starts the TiCProxy loop.
 * 
 * \par
 * By default:
 * - the node tries to attach to the CNBI pipe "/ctrl10" as reader
 * - the node tries to attach to the CNBI pipe "/ctrl11" as writer
 * - the node subscribe on the ROS topic "/tic_to_cnbiloop" and if
 *   there is an active connection to "/ctrl11", it forwards the message to such
 *   a pipe
 * - the node publishes any message arriving from the loop on "/ctrl10" to
 *   the ROS topic "/ticproxy" 
 * 
 * \par XML launcher:
 * It is possible to configure the following parameters in the XML launcher:
 * - \p ~cnbiloop 	(\p str, default: none) \n 
 * 	 The address of the CNBI Loop.
 * - \p ~frompipes	(\p str[], default: /ctrl10) \n
 *   Vector of strings with the names of the reading pipes.
 * - \p ~topipes 	(\p str[], default: /ctrl11) \n
 *   Vector of strings with the names of the writing pipes. For each pipe, a
 *   corresponding subtopic must exists (see below).
 * - \p ~subtopics 	(\p str[], default: /tic_to_cnbiloop) \n
 *   Vector of strings with the names of the subscribed topics. They are
 *   considered in the same order of the \a ~topipes names provided.
 *
 * \sa cnbiros::bci::TiCProxy(), tobi_interface_d.cpp
 *
 */ 
int main(int argc, char** argv) {

	std::string 			 address;						
	std::vector<std::string> read_pipes; 		
	std::vector<std::string> write_pipes;  	
	std::vector<std::string> topics;	

	// ROS initialization
	ros::init(argc, argv, "tobi_interface_c");

	// Node initialization
	cnbiros::bci::CnbiInterface 	cnbiloop;
	cnbiros::bci::TiCProxy 		proxy;
	
	// Get parameters from server (with default values)
	proxy.getParam("address", 	 	address);
	proxy.getParam("read_pipes", 	read_pipes);
	proxy.getParam("write_pipes",	write_pipes);
	proxy.getParam("topics", 	 	topics);
	
	// Check the size of the writing pipes and the subtopics
	if(write_pipes.size() != topics.size()) {
		ROS_FATAL("%s reports an error during initialization: number of writing pipes"
				  "and subscribe topics is different. Shutting down the node", 
				  ros::this_node::getName().c_str()); 
		proxy.shutdown();
		exit(EXIT_FAILURE);
	}

	// Connect to the cnbiloop (blocking);
	ROS_INFO("Try to connect to the CNBI Loop: %s", cnbiloop.GetAddress().c_str());
	cnbiloop.Connect();


	// Attach to all the provided reading pipes
	for(auto itr = read_pipes.begin(); itr != read_pipes.end(); ++itr) {
		if(proxy.Attach(cnbiros::bci::TiCProxy::AsReader, *itr) == true) {
			ROS_INFO("%s attached to pipe \'%s\' as reader", proxy.GetName().c_str(), (*itr).c_str());
		} else {
			ROS_WARN("%s cannot attach to pipe \'%s\' as reader", proxy.GetName().c_str(), (*itr).c_str());
		}
	}

	// Attach to all the provided writing pipes
	for(auto itw = write_pipes.begin(), itt = topics.begin(); itw != write_pipes.end(); ++itw, ++itt) {
		if(proxy.Attach(cnbiros::bci::TiCProxy::AsWriter, *itw, *itt) == true) {
			ROS_INFO("%s attached to pipe \'%s\' as writer and subscribed on topic \'%s\'", 
					 proxy.GetName().c_str(), (*itw).c_str(), (*itt).c_str());
		} else {
			ROS_WARN("%s cannot attach to pipe \'%s\' as writer", 
					  proxy.GetName().c_str(), (*itw).c_str());
		}
	}

	// Run the TiC proxy 
	proxy.Run();

	// Spin last time before exit
	ros::spin();

	return EXIT_SUCCESS;
}
