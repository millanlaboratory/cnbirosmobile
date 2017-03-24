#ifndef CNBIROS_CORE_MOTOR_CPP
#define CNBIROS_CORE_MOTOR_CPP

#include "cnbiros_core/Motor.hpp"

namespace cnbiros {
	namespace core {

Motor::Motor(std::string name) : RosInterface(name) {

	// Abstract sensor initialization
	this->SetSubscriber("/cmd_vel", &Motor::onReceived, this);

	// Service for velocity set
	this->srv_twist_ = this->advertiseService("set_twist", &Motor::on_service_settwist_, this);

}

Motor::~Motor(void) {}

void Motor::onReceived(const geometry_msgs::Twist& msg) {

	this->motor_twist_ = msg;
	this->SetVelocity(msg);
}

void Motor::GetVelocity(geometry_msgs::Twist& twist) {
	twist.linear  = this->motor_twist_.linear;
	twist.angular = this->motor_twist_.angular;
}


bool Motor::on_service_settwist_(cnbiros_services::SetTwist::Request&  req,
							  	 cnbiros_services::SetTwist::Response& res) {
	
	res.result = true;
	this->motor_twist_.linear  = req.linear;
	this->motor_twist_.angular = req.angular;
	return res.result;
}


	}
}

#endif
