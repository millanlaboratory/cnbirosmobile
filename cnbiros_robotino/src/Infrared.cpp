#ifndef CNBIROS_ROBOTINO_INFRARED_CPP
#define CNBIROS_ROBOTINO_INFRARED_CPP

#include "cnbiros_robotino/Infrared.hpp"

namespace cnbiros {
	namespace robotino {

Infrared::Infrared(std::string hostname, std::string name) 
					: core::Sensor<sensor_msgs::PointCloud>(name) {

	// Connection to the robot
	this->com_ = new Communication(this->GetName());
	this->com_->Connect(hostname);
	
	// Create infrared sensor association
	this->setComId(this->com_->id());
}

Infrared::~Infrared(void) {
	this->com_->Disconnect();
	delete this->com_;
}

void Infrared::distancesChangedEvent(const float* ranges, unsigned int size) {

	float x, y, z;
	float angle_inc, range_min, range_max, height, base_radius;

	angle_inc   = CNBIROS_ROBOTINO_INFRARED_ANGLE_INC;
	range_max   = CNBIROS_ROBOTINO_INFRARED_RANGE_MAX;
	height 		= CNBIROS_ROBOTINO_INFRARED_HEIGHT;
	base_radius = CNBIROS_ROBOTINO_BASE_RADIUS;

	// Build the PointCloud msg
	this->sensor_data_.header.stamp = ros::Time::now();
	this->sensor_data_.header.frame_id = "base_link";
	this->sensor_data_.points.resize(size);

	// Fill the point cloud by iterating the input vector of distance. Checking
	// if the distance are inside the min-max range of the infrared sensors.
	// Otherwise set the value to 0 (not done by the robotino::api2)
	for(auto i = 0; i < size; ++i) {
		x = 0, y = 0, z = 0;	
		if(ranges[i] < range_max) {
			x = (ranges[i] + base_radius) * cos(angle_inc * i);
			y = (ranges[i] + base_radius) * sin(angle_inc * i);
			z = height;
		} 
		
		this->sensor_data_.points[i].x = x;
		this->sensor_data_.points[i].y = y;
		this->sensor_data_.points[i].z = z;
	}

	this->Publish(this->rostopic_, this->sensor_data_);
}

void Infrared::Reset(void) {

	for(auto it = this->sensor_data_.points.begin(); it != this->sensor_data_.points.end(); ++it) {
		(*it).x = 0.0f, (*it).y = 0.0f, (*it).z = 0.0f;
	}
}

void Infrared::onRunning(void) {
	
	// Process robotino infrared events (via api2 callback)
	if(this->com_->IsConnected()) {
		this->com_->processEvents();
	}
}

	}
}

#endif
