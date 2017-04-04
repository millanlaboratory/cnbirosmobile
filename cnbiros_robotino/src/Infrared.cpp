#ifndef CNBIROS_ROBOTINO_INFRARED_CPP
#define CNBIROS_ROBOTINO_INFRARED_CPP

#include "cnbiros_robotino/Infrared.hpp"

namespace cnbiros {
	namespace robotino {

Infrared::Infrared(std::string name) 
					: core::Sensor<sensor_msgs::PointCloud>(name) {

	// Connection to the robot
	this->com_ = new Communication(this->GetName());
}

Infrared::~Infrared(void) {
	this->com_->Disconnect();
	delete this->com_;
}

bool Infrared::Connect(std::string hostname, bool blocking) {

	do {
		try {
			this->com_->Connect(hostname);
		} catch (rec::robotino::api2::RobotinoException &e) {};

		ros::Duration(1.0f).sleep();
	} while((this->com_->IsConnected() == false) && (blocking == true));

	// If connected associate to comid
	if(this->com_->IsConnected()) {
		this->setComId(this->com_->id());
	}

	return this->com_->IsConnected();
}

void Infrared::distancesChangedEvent(const float* ranges, unsigned int size) {

	//float x, y, z;
	float angle_inc, range_min, range_max, height, base_radius;
	geometry_msgs::Point32 point;
	sensor_msgs::ChannelFloat32 channel;

	angle_inc   = CNBIROS_ROBOTINO_INFRARED_ANGLE_INC;
	range_max   = CNBIROS_ROBOTINO_INFRARED_RANGE_MAX;
	height 		= CNBIROS_ROBOTINO_INFRARED_HEIGHT;
	base_radius = CNBIROS_ROBOTINO_BASE_RADIUS;

	// Build the PointCloud msg
	this->sensor_data_.header.stamp = ros::Time::now();
	this->sensor_data_.header.frame_id = "base_link";
	this->sensor_data_.points.clear();
	this->sensor_data_.channels.clear();

	

	channel.name = "strength";
	channel.values.resize(size);

	// Fill the point cloud by iterating the input vector of distance. Checking
	// if the distance are inside the min-max range of the infrared sensors.
	// Otherwise set the value to 0 (not done by the robotino::api2)
	for(auto i = 0; i < size; ++i) {
		point.x = (ranges[i] + base_radius) * cos(angle_inc * i);
		point.y = (ranges[i] + base_radius) * sin(angle_inc * i);
		point.z = height;
		
		channel.values[i] = 0.0f;
		if(ranges[i] > 0.0f && ranges[i] < range_max) {
			channel.values[i] = 1.0f;
		} 

		this->sensor_data_.points.push_back(point);
	}
	this->sensor_data_.channels.push_back(channel);

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
