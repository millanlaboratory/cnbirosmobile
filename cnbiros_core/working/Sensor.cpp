#ifndef SENSOR_CPP
#define SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(ros::NodeHandle* node) {
	this->rosnode_   = node;

	// Set default frequency
	this->SetFrequency(CNBIROS_NODE_FREQUENCY);
}

Sensor::~Sensor(void){}

void Sensor::SetFrequency(float frequency) {
	this->frequency_ = frequency;
	this->rosrate_ 	 = new ros::Rate(this->frequency_);
}

float Sensor::GetFrequency(void) {
	return this->frequency_;
}

void Sensor::SetFrames(std::string frameid, std::string child_frameid) {

	this->frameid_ 		 = frameid;
	this->child_frameid_ = child_frameid;
}

void Sensor::SetGrid(std::string layer, float x, float y, float r) {

	// Configuration of Grid map
	this->grid_layer_ = layer;
	this->grid_.add(this->grid_layer_, 0.0f);
	this->grid_.setFrameId(this->frameid_);
	this->grid_.setGeometry(grid_map::Length(x, y), r);
	this->grid_[this->grid_layer_].setConstant(0.0);
}

	}
}

#endif
