#ifndef SENSOR_CPP
#define SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(std::string name){
	this->rosnode_   = nullptr;
	this->rostopic_  = "";
	this->rosname_   = name;
	this->rosrate_   = nullptr;
	this->frequency_ = 1;
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
		this->rospub_   = this->rosnode_->advertise<grid_map_msgs::GridMap>(this->rostopic_, SENSOR_BUFFER_SIZE);
	else
		ROS_ERROR("Can't advertise: object is not registered to any node");

}

void Sensor::SetGridParameters(std::string layer, std::string frameid) {
	this->grid_base_layer_ 	 = layer;
	this->grid_base_frameid_ = frameid;
}

void Sensor::SetGridDimensions(float xdim, float ydim, float resolution) {
	this->grid_xdim_ 		= xdim;
	this->grid_ydim_ 	   	= ydim;
	this->grid_resolution_ 	= resolution;
}

	}
}

#endif
