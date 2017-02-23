#ifndef SENSORSFUSION_CPP
#define SENSORSFUSION_CPP

#include "SensorsFusion.hpp"

namespace cnbiros {
	namespace core {

SensorsFusion::SensorsFusion(float frequency) {
	this->rosnode_ 	 = nullptr;
	this->frequency_ = frequency;
	this->rosrate_   = new ros::Rate(frequency);
};

SensorsFusion::~SensorsFusion(void) {};

void SensorsFusion::Register(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

bool SensorsFusion::IsRegistered(void) {
	if(this->rosnode_ != nullptr)
		return true;
	else
		return false;
}

void SensorsFusion::Advertise(std::string topic) {

	this->rospub_topic_ = topic;
	if(this->IsRegistered()) {
		this->rospub_ = this->rosnode_->advertise<grid_map_msgs::GridMap>(this->rospub_topic_, CNBIROS_MESSAGES_BUFFER);
	} else {
		ROS_ERROR("Can't advertise on %s: object is not registered to any node", topic.c_str());
	}
}

void SensorsFusion::Subscribe(std::string topic) {
	
	// Register/replace new subscriber 
	if(this->IsRegistered()) {
		this->rossublist_[topic] = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, &SensorsFusion::add_layer_callback, this);
	} else {
		ROS_ERROR("Can't subscribe to %s: object is not registered to any node", topic.c_str());
	}
}

void SensorsFusion::SetGrid(std::string layer, std::string frameid) {
	this->layer_	 = layer;
	this->frameid_	 = frameid;
	
	// Configuration of Grid map - Layer: fusion
	this->grid_.add(this->layer_, 0.0f);
	this->grid_.setFrameId(this->frameid_);
}

void SensorsFusion::SetGrid(float x, float y, float r) {
	this->grid_.setGeometry(grid_map::Length(x, y), r);
	this->grid_[this->layer_].setConstant(0.0);
}

void SensorsFusion::add_layer_callback(const grid_map_msgs::GridMap& msg) {

	grid_map::GridMap grid;

	// Convert grid message to the grid object 
	if(grid_map::GridMapRosConverter::fromMessage(msg, grid) == false)
		ROS_WARN("Cannot convert message to grid");

	// Add all layers from grid message to the local grid 
	this->grid_.addDataFrom(grid, true, true, true);
}

void SensorsFusion::fuse_layers(std::string target) {

	std::vector<std::string> layers;
	std::vector<std::string>::iterator it;

	// Get all layers in the local grid
	layers = this->grid_.getLayers();

	// Reset the local grid
	this->grid_[target].setConstant(0.0);

	// Iterate and sum layers to target layer
	for (it = layers.begin(); it != layers.end(); it++) {
		this->grid_[target] += this->grid_[*it];
	}

	// Normalization with respect to the maximum [0 1]
	this->grid_[target] /= this->grid_[target].maxCoeff();

}

void SensorsFusion::Run(void) {
	
	while(this->rosnode_->ok()) {

		// Fuse layers
		this->fuse_layers(this->layer_);

		// Publish the msg
		grid_map::GridMapRosConverter::toMessage(this->grid_, this->grid_msg_);
		this->rospub_.publish(this->grid_msg_);
		
		ros::spinOnce();
		this->rosrate_->sleep();
	}
}


	}
}

#endif
