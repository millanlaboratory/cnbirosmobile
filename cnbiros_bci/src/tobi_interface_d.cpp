#include <ros/ros.h>

#include "cnbiros_bci/CnbiInterface.hpp"
#include "cnbiros_bci/TiDProxy.hpp"

/*! 
 * \file
 *
 * \brief Tobi interface D proxy node
 *
 * \par General description:
 * This executable provides an implementation of a working TiD proxy node. It
 * can be used as it is and it can be configured via xml launch file.
 *
 * \par Main behavior:
 * This node (namely 'tobi_interface_d') connects to the CNBI Loop (waiting
 * until the connection succed), attached to the provided pipes (always in
 * reader/writer modality) and starts the TiDProxy loop.
 * 
 * \par
 * By default:
 * - the node tries to attach to the CNBI pipe "/bus" as reader/writer
 * - the node tries to attach to the CNBI pipe "/dev" as reader/writer
 * - the node subscribe on the ROS topic "/tid_ros2bus" and if
 *   there is an active connection to "/bus", it forwards the message to such
 *   a pipe
 * - the node subscribe on the ROS topic "/tid_ros2dev" and if
 *   there is an active connection to "/dev", it forwards the message to such
 *   a pipe
 * - the node publishes any message arriving from the loop on "/bus" and "/dev"
 * to the ROS topic "/tidproxy" 
 * 
 * \par XML launcher:
 * It is possible to configure the following parameters in the XML launcher:
 * - \p ~cnbiloop 	(\p str, default: none) \n 
 * 	 The address of the CNBI Loop.
 * - \p ~pipes		(\p str[], default: [/bus, /dev]) \n
 *   Vector of strings with the names of the reading/writing pipes.
 * - \p ~subtopics 	(\p str[], default: [/tid_ros2bus, /tid_ros2dev]) \n
 *   Vector of strings with the names of the subscribed topics. They are
 *   considered in the same order of the \a ~pipes names provided.
 *
 * \sa cnbiros::bci::TiDProxy(), tobi_interface_c.cpp
 *
 */ 

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

	// Connect to the cnbiloop (blocking);
	ROS_INFO("Try to connect to the CNBI Loop (%s)", cnbiloop->GetAddress().c_str());
	cnbiloop->Connect();
	
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
