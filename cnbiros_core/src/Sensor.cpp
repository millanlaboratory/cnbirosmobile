#ifndef SENSOR_CPP
#define SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(float frequency){
	this->rosnode_   = nullptr;
	this->rostopic_  = "";
	this->rosrate_   = nullptr;
	this->frequency_ = frequency;
}

Sensor::~Sensor(void){}

void Sensor::Register(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

bool Sensor::IsRegistered(void) {
	if(this->rosnode_ != nullptr)
		return true;
	else
		return false;
}
void Sensor::Advertise(std::string topic) {
	this->rostopic_ = topic;
	if(this->IsRegistered())
		this->rospub_ = this->rosnode_->advertise<grid_map_msgs::GridMap>(this->rostopic_, CNBIROS_MESSAGES_BUFFER);
	else
		ROS_ERROR("Can't advertise: object is not registered to any node");

}

void Sensor::SetGrid(std::string layer, std::string frame) {
	
	this->layer_   = layer;
	this->frameid_ = frame;
	
	// Configuration of Grid map - Layer: fusion
	this->grid_.add(this->layer_, 0.0f);
	this->grid_.setFrameId(this->frameid_);
}

void Sensor::SetGrid(float x, float y, float r) {
	this->grid_.setGeometry(grid_map::Length(x, y), r);
	this->grid_[this->layer_].setConstant(0.0);
}

	}
}

#endif
