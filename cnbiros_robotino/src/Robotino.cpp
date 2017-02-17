#ifndef ROBOTINO_CPP
#define ROBOTINO_CPP

#include "Robotino.hpp"

namespace cnbiros {
	namespace robotino {

Robotino:: Robotino(std::string hostname, 
					std::string name, 
					std::string id) : 
					core::Robot(core::Robot::Type::Robotino, name, id) {
	
	// Initialize motor velocities
	this->vx_ = 0.0f;
	this->vy_ = 0.0f;
	this->vo_ = 0.0f;

	// Initialize connection to the base
	this->com_ = new RobotinoCom(name);

	ROS_INFO("%s tries to connect to %s...", name.c_str(), hostname.c_str());
	this->com_->Connect(hostname);

	// If connected associate omnidrive to comid
	this->omnidrive_.setComId(this->com_->id());
}

Robotino::~Robotino(void) {
	this->com_->Disconnect();
}

bool Robotino::IsConnected(void) {
	return this->com_->IsConnected();
}


void Robotino::velocityCallback(const cnbiros_messages::RobotVelocity& msg) {

	this->vx_ = msg.vx;
	this->vy_ = msg.vy;
	this->vo_ = msg.vo;
	ROS_INFO("New velocity requested: vx=%f, vy=%f, vo=%f", this->vx_, this->vy_, this->vo_);
}

int Robotino::Run(void) {
	this->omnidrive_.setVelocity(this->vx_, this->vy_, this->vo_);	
	this->com_->processEvents();
	return 0;
}

int Robotino::Stop(void) {
	return 0;
}

	}
}

#endif
