#include <ros/ros.h>

#include "RobotinoInfrared.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	std::string hostname, topic, grid_frame, grid_layer;
	float frequency, decay, grid_x, grid_y, grid_r;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_infrared");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Retrieve configuration parameters
	if(ros::param::get("/frequency", frequency) == false)
		ROS_ERROR("Parameter frequency not found");
	if(ros::param::get("/hostname", hostname) == false)
		ROS_ERROR("Parameter hostname not found");
	if(ros::param::get("/robotino_node_infrared/topic", topic) == false)
		ROS_ERROR("Parameter topic not found");
	if(ros::param::get("/robotino_node_infrared/decay", decay) == false)
		ROS_ERROR("Parameter decay not found");
	if(ros::param::get("/grid_x", grid_x) == false)
		ROS_ERROR("Parameter grid_x not found");
	if(ros::param::get("/grid_y", grid_y) == false)
		ROS_ERROR("Parameter grid_y not found");
	if(ros::param::get("/grid_r", grid_r) == false)
		ROS_ERROR("Parameter grid_r not found");
	if(ros::param::get("/grid_frame", grid_frame) == false)
		ROS_ERROR("Parameter grid_frame not found");
	if(ros::param::get("/robotino_node_infrared/grid_layer", grid_layer) == false)
		ROS_ERROR("Parameter grid_layer not found");
	
	// Create sensor instance
	RobotinoInfrared* infrared;
	infrared = new RobotinoInfrared(hostname, frequency);
	
	// Configure infrared grid and decay rate
	infrared->SetGrid(grid_layer, grid_frame);
	infrared->SetGrid(grid_x, grid_y, grid_r);
	infrared->SetDecayTime(decay);

	// Register and advertise topics
	infrared->Register(&node);
	infrared->Advertise(topic);

	// Run loop
	infrared->Process();
		
	ros::spin();
		
	delete infrared;

	return 0;
}
