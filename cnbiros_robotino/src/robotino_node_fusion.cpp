#include <ros/ros.h>
#include "SensorsFusion.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	float frequency, grid_x, grid_y, grid_r;
	std::string topic, grid_frame, grid_layer;

	// ROS initialization
	ros::init(argc, argv, "robotino_node_fusion");
	
	// Create node handler
	ros::NodeHandle node("~");

	// Retrieve configuration parameters
	if(ros::param::get("/frequency", frequency) == false)
		ROS_ERROR("Parameter frequency not found");
	if(ros::param::get("/robotino_node_fusion/topic", topic) == false)
		ROS_ERROR("Parameter topic not found");
	if(ros::param::get("/grid_x", grid_x) == false)
		ROS_ERROR("Parameter grid_x not found");
	if(ros::param::get("/grid_y", grid_y) == false)
		ROS_ERROR("Parameter grid_y not found");
	if(ros::param::get("/grid_r", grid_r) == false)
		ROS_ERROR("Parameter grid_r not found");
	if(ros::param::get("/grid_frame", grid_frame) == false)
		ROS_ERROR("Parameter grid_frame not found");
	if(ros::param::get("/robotino_node_fusion/grid_layer", grid_layer) == false)
		ROS_ERROR("Parameter grid_layer not found");

	// Create fusion object
	SensorsFusion* 	fusion;
	fusion = new SensorsFusion(frequency);

	// Configure fusion grid
	fusion->SetGrid(grid_layer, grid_frame);
	fusion->SetGrid(grid_x, grid_y, grid_r);

	// Register to the node and advertise/subscribe to topics
	fusion->Register(&node);
	fusion->Advertise(topic);
	fusion->Subscribe("/sensor_infrared");
	fusion->Subscribe("/sensor_kinect");
	fusion->Subscribe("/sensor_sonar");

	// Run main loop
	fusion->Run();

	ros::spin();

	delete fusion;

	return 0;
}
