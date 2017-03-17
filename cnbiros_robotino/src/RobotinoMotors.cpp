#ifndef CNBIROS_ROBOTINO_MOTORS_CPP
#define CNBIROS_ROBOTINO_MOTORS_CPP

#include "RobotinoMotors.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoMotors:: RobotinoMotors(std::string hostname, std::string name) : core::Motors(name) {

	// Default values
	this->hostname_     = hostname;
	
	// Initialize motor velocities
	this->rostwist_msg_.linear.x  = 0.0f;
	this->rostwist_msg_.linear.y  = 0.0f;
	this->rostwist_msg_.linear.z  = 0.0f;
	this->rostwist_msg_.angular.x = 0.0f;
	this->rostwist_msg_.angular.y = 0.0f;
	this->rostwist_msg_.angular.z = 0.0f;

	// Connection to the base
	ROS_INFO("Robotino %s tries to connect to the base (%s)...", 
			this->GetName().c_str(), this->hostname_.c_str());
	this->com_ = new RobotinoCom(this->GetName());
	this->com_->Connect(this->hostname_);

	// If connected associate omnidrive to comid
	this->omnidrive_.setComId(this->com_->id());
}

RobotinoMotors::~RobotinoMotors(void) {}


void RobotinoMotors::onRunning(void) {
	
	this->omnidrive_.setVelocity(this->rostwist_msg_.linear.x,
								 this->rostwist_msg_.linear.y,
								 this->rostwist_msg_.angular.z);	
	this->com_->processEvents();
}

	}
}

#endif
