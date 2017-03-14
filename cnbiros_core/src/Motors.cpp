#ifndef CNBIROS_CORE_MOTORS_CPP
#define CNBIROS_CORE_MOTORS_CPP

#include "Motors.hpp"

namespace cnbiros {
	namespace core {

Motors::Motors(std::string name) {

	// Abstract sensor initialization
	this->SetName(name);
	this->SetSubscriber(CNBIROS_ROBOTBASE_TOPIC, &Motors::rosvelocity_callback, this);

	// Service for velocity set
	this->rossrv_velocity_ = this->advertiseService("velocity_set", 
											&Motors::on_set_velocity_, this);
}

Motors::~Motors(void) {}

void Motors::rosvelocity_callback(const geometry_msgs::Twist& msg) {
	this->vx_ = msg.linear.x;
	this->vy_ = msg.linear.y;
	this->vz_ = msg.linear.z;
	this->vo_ = msg.angular.z;
}

bool Motors::on_set_velocity_(cnbiros_services::SetBaseVelocity::Request& req,
							 	cnbiros_services::SetBaseVelocity::Response& res) {
	
	res.result = true;

	this->vx_ = req.vx;
	this->vy_ = req.vy;
	this->vz_ = req.vz;
	this->vo_ = req.vo;

	return res.result;
}


	}
}

#endif
