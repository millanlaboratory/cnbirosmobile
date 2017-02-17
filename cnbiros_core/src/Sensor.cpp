#ifndef SENSOR_CPP
#define SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(std::string name){
	this->rosnode_  = nullptr;
	this->rostopic_ = "";
	this->rosname_  = name;
}

Sensor::~Sensor(void){}

void Sensor::Advertise(ros::NodeHandle* node, std::string topic) {
	this->rosnode_  = node;	
	this->rostopic_ = topic;
	this->rospub_   = this->rosnode_->advertise<grid_map_msgs::GridMap>(this->rostopic_, SENSOR_BUFFER_SIZE);
}

	}
}

#endif
