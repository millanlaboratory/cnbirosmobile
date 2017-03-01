#ifndef CNBIROS_CORE_SENSOR_CPP
#define CNBIROS_CORE_SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(ros::NodeHandle* node) {

	this->rosnode_ = node;

	// Default initialization
	this->SetName("sensor");
	this->SetFrequency(CNBIROS_NODE_FREQUENCY);
}

Sensor::~Sensor(void) {};

void Sensor::SetName(std::string name) {
	this->name_ = name;
}

void Sensor::SetFrequency(float frequency) {
	this->frequency_ = frequency;
	this->rosrate_   = new ros::Rate(this->frequency_);
}

void Sensor::SetPublisher(std::string topic) {

	this->rospub_list_[topic] = this->rosnode_->advertise<grid_map_msgs::GridMap>
								(topic, CNBIROS_MESSAGES_BUFFER);
	ROS_INFO("%s advertises topic %s", this->name_.c_str(), topic.c_str());
}

void Sensor::SetGrid(std::string layer, float xsize, float ysize, float res, std::string frame) {

	this->rosgrid_.add(layer, 0.0f);
	this->rosgrid_.setFrameId(frame);
	this->rosgrid_.setGeometry(grid_map::Length(xsize, ysize), res);
	this->rosgrid_[layer].setConstant(0.0f);

	this->rosgrid_layer_ = layer;
	this->rosgrid_frame_ = frame;
}

void Sensor::GetName(std::string& name) {
	name = this->name_;
}

void Sensor::GetFrequency(float& frequency) {
	frequency = this->frequency_;
}

void Sensor::GetPublishTopics(std::vector<std::string>& topics) {

	std::map<std::string, ros::Publisher>::iterator it;
	
	topics.clear();
	for(it = this->rospub_list_.begin(); it != this->rospub_list_.end(); ++it) {
		topics.push_back(it->first);
	}
}

void Sensor::GetPublisher(ros::Publisher& publisher, std::string topic) {
	std::map<std::string, ros::Publisher>::iterator it;

	it = this->rospub_list_.find(topic);

	if(it != this->rospub_list_.end()) {
		publisher = it->second;
	} else {
		ROS_WARN("Publisher %s does not exist for sensor %s", topic.c_str(), this->name_.c_str());
	}
}

void Sensor::GetGrid(grid_map::GridMap& grid) {
	grid = this->rosgrid_;
}

void Sensor::DeletePublisher(std::string topic) {
	std::map<std::string, ros::Publisher>::iterator it;

	it = this->rospub_list_.find(topic);

	if(it != this->rospub_list_.end()) {
		this->rospub_list_.erase(it);
	}
}

void Sensor::ResetLayer(std::string layer, float value) {
	this->rosgrid_[layer].setConstant(value);
}

void Sensor::ReplaceNaN(grid_map::GridMap& grid, float value) {

	std::vector<std::string> layers;
	std::vector<std::string>::iterator it;

	layers = grid.getLayers();

	for (it = layers.begin(); it != layers.end(); ++it) {
		grid_map::Matrix& data = grid[*it];
		data = (data.array().isNaN()).select(0.0f, data);
	}
}


void Sensor::PublishGrid(void) {

	std::map<std::string, ros::Publisher>::iterator it;
	
	// Convert grid map to grid message
	grid_map::GridMapRosConverter::toMessage(this->rosgrid_, this->rosgrid_msg_);

	// Publish the grid message on all the advertised topics
	for (it = this->rospub_list_.begin(); it != this->rospub_list_.end(); ++it) {
		it->second.publish(this->rosgrid_msg_);
	}
}

	}
}



#endif
