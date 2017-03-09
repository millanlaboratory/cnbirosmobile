#ifndef ROBOTINO_CPP
#define ROBOTINO_CPP

#include "RobotinoBase.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoBase:: RobotinoBase(std::string hostname, ros::NodeHandle* node, std::string name) : core::RobotBase(node, name) {

	// Default values
	this->hostname_     = hostname;
	this->SetName("base");
	
	// Initialize motor velocities
	this->vx_ = 0.0f;
	this->vy_ = 0.0f;
	this->vo_ = 0.0f;

	// Connection to the base
	ROS_INFO("Robotino %s tries to connect to the base (%s)...", 
			this->GetName().c_str(), this->hostname_.c_str());
	this->com_ = new RobotinoCom(this->GetName());
	this->com_->Connect(this->hostname_);

	// If connected associate omnidrive to comid
	this->omnidrive_.setComId(this->com_->id());
}

RobotinoBase::~RobotinoBase(void) {}


void RobotinoBase::onRunning(void) {
	
	this->omnidrive_.setVelocity(this->vx_, this->vy_, this->vo_);	
	this->com_->processEvents();
}

	}
}

#endif
