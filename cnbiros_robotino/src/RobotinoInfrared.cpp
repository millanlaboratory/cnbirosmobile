#ifndef ROBOTINOINFRARED_CPP
#define ROBOTINOINFRARED_CPP

#include "RobotinoInfrared.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoInfrared::RobotinoInfrared(std::string hostname, 
								   float frequency) : Sensor(frequency) {

	// Initialize parameters
	this->rosrate_ = new ros::Rate(frequency);

	// Connection to the base
	ROS_INFO("Robotino infrared tries to connect to the base (%s)...", hostname.c_str());
	this->com_ = new RobotinoCom("infrared");
	this->com_->Connect(hostname);

	// Default decay rate (instantaneous)
	this->decayrate_ = 0.0f;

	// Create infrared sensor association
	this->setComId(this->com_->id());
}

void RobotinoInfrared::SetDecayTime(float time) {
	this->decayrate_ = 1.0f/float(this->frequency_ * time);
}

void RobotinoInfrared::distancesChangedEvent(const float* distances, unsigned int size) {

	float x, y;

	// In case the decay is set to 0, clear the grid once the callback is called
	if(this->decayrate_ == 0) {
		this->grid_[this->layer_].setConstant(0.0);
	}
	
	// Iterate along infrared sensors
	for(auto i=0; i<size; i++) {
		x = (distances[i]+CNBIROS_ROBOTINO_RADIUS)*cos(CNBIROS_ROBOTINO_INFRARED_ANGLE*i);
		y = (distances[i]+CNBIROS_ROBOTINO_RADIUS)*sin(CNBIROS_ROBOTINO_INFRARED_ANGLE*i);
		
		grid_map::Position position(x, y);
		
		if (distances[i] < CNBIROS_ROBOTINO_INFRARED_MAXDISTANCE ||
			distances[i] > CNBIROS_ROBOTINO_INFRARED_MINDISTANCE) {
			this->grid_.atPosition(this->layer_, position) = exp(-(distances[i] - CNBIROS_ROBOTINO_INFRARED_MINDISTANCE));
		}

		if (distances[i] >= CNBIROS_ROBOTINO_INFRARED_MAXDISTANCE)
			this->grid_.atPosition(this->layer_, position) = 0.0f;
	}
}

void RobotinoInfrared::ProcessDecay(void) {
	// Substract the decay step from the grid
	this->grid_[this->layer_].array() -= this->decayrate_;

	// Set to 0 every negative value
	for (grid_map::GridMapIterator it(this->grid_); !it.isPastEnd(); ++it) {
		if(this->grid_.at(this->layer_, *it) < 0)
			this->grid_.at(this->layer_, *it) = 0.0f;
    }
}

void RobotinoInfrared::Process(void) {
	
	while(this->rosnode_->ok()) {
		
		// Process robotino infrared events (via api2 callback)
		this->com_->processEvents();

		// Process map decay
		this->ProcessDecay();
		
		// Publish the msg
		grid_map::GridMapRosConverter::toMessage(this->grid_, this->grid_msg_);
		this->rospub_.publish(this->grid_msg_);

		this->rosrate_->sleep();
		ros::spinOnce();
	}
}

	}
}

#endif
