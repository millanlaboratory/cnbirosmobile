#ifndef CNBIROS_ROBOTINO_MOTOR_CPP
#define CNBIROS_ROBOTINO_MOTOR_CPP

#include "cnbiros_robotino/Motor.hpp"

namespace cnbiros {
	namespace robotino {

Motor:: Motor(std::string name) : core::Motor(name) {

	// Connection to the robot
	this->com_ = new Communication(this->GetName());
}

Motor::~Motor(void) {
	this->com_->Disconnect();
	delete this->com_;
}

bool Motor::Connect(std::string hostname, bool blocking) {

	do {
		try {
			this->com_->Connect(hostname);
		} catch (rec::robotino::api2::RobotinoException &e) {};

		ros::Duration(1.0f).sleep();
	} while((this->com_->IsConnected() == false) && (blocking == true));

	// If connected associate to comid
	if(this->com_->IsConnected()) {
		this->omnidrive_.setComId(this->com_->id());
	}

	return this->com_->IsConnected();
}

void Motor::SetVelocity(const geometry_msgs::Twist& twist) {
	this->motor_twist_ = twist;
	this->omnidrive_.setVelocity(this->motor_twist_.linear.x,
								 this->motor_twist_.linear.y,
								 this->motor_twist_.angular.z);
}

void Motor::onRunning(void) {

	this->SetVelocity(this->motor_twist_);
	this->com_->processEvents();
}

	}
}

#endif
