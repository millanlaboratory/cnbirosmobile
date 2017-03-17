#ifndef CNBIROS_CORE_MOTORS_CPP
#define CNBIROS_CORE_MOTORS_CPP

#include "Motors.hpp"

namespace cnbiros {
	namespace core {

Motors::Motors(std::string name) : RosInterface(name) {

	// Abstract sensor initialization
	this->SetSubscriber(CNBIROS_MOTORS_TOPIC, &Motors::rostwist_callback_, this);

	// Service for velocity set
	this->rossrv_twist_ = this->advertiseService("set_twist", 
										&Motors::on_service_settwist_, this);
}

Motors::~Motors(void) {}

void Motors::rostwist_callback_(const geometry_msgs::Twist& msg) {

	this->rostwist_msg_ = msg;
}

bool Motors::on_service_settwist_(cnbiros_services::SetTwist::Request&  req,
							  	  cnbiros_services::SetTwist::Response& res) {
	
	res.result = true;
	this->rostwist_msg_.linear  = req.linear;
	this->rostwist_msg_.angular = req.angular;
	return res.result;
}


	}
}

#endif
