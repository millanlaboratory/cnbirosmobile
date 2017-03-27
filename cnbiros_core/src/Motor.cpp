#ifndef CNBIROS_CORE_MOTOR_CPP
#define CNBIROS_CORE_MOTOR_CPP

#include "cnbiros_core/Motor.hpp"

namespace cnbiros {
	namespace core {

Motor::Motor(std::string name) : RosInterface(name) {

	// Abstract sensor initialization
	this->topic_ = "/cmd_vel";
	this->SetSubscriber(this->topic_, &Motor::onReceived, this);

	// Services for velocity set/reset
	this->srv_set_   = this->advertiseService("set_twist", &Motor::on_service_set_, this);
	this->srv_reset_ = this->advertiseService("reset_twist", &Motor::on_service_reset_, this);

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

void Motor::Reset(void) {
	this->motor_twist_.linear.x  = 0.0f;
	this->motor_twist_.linear.y  = 0.0f;
	this->motor_twist_.linear.z  = 0.0f;
	this->motor_twist_.angular.x = 0.0f;
	this->motor_twist_.angular.y = 0.0f;
	this->motor_twist_.angular.z = 0.0f;
	this->SetVelocity(this->motor_twist_);
}

void Motor::onStart(void) {
	ROS_INFO("%s has been required to start", this->GetName().c_str());
	this->Reset();
}

void Motor::onStop(void) {
	ROS_INFO("%s has been required to stop", this->GetName().c_str());
	this->Reset();
}


bool Motor::on_service_set_(cnbiros_services::SetTwist::Request&  req,
							cnbiros_services::SetTwist::Response& res) {
	
	res.result = true;
	this->motor_twist_.linear  = req.linear;
	this->motor_twist_.angular = req.angular;
	this->SetVelocity(this->motor_twist_);
	this->Publish(this->topic_, this->motor_twist_);
	
	return res.result;
}

bool Motor::on_service_reset_(cnbiros_services::Reset::Request& req,
							  cnbiros_services::Reset::Response& res) {

	res.result = true;
	this->Reset();
	this->Publish(this->topic_, this->motor_twist_);

	return res.result;
}
	}
}

#endif
