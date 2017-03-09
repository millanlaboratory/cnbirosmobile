#ifndef ROBOT_CPP
#define ROBOT_CPP

#include "RobotBase.hpp"

namespace cnbiros {
	namespace core {

RobotBase::RobotBase(ros::NodeHandle* node, std::string name) : RosInterface(node) {

	// Abstract sensor initialization
	this->SetName(name);
	this->SetSubscriber(CNBIROS_ROBOTBASE_TOPIC, &RobotBase::rosvelocity_callback, this);

	// Service for velocity set
	this->rossrv_velocity_ = this->rosnode_->advertiseService("velocity_set", 
											&RobotBase::on_set_velocity_, this);
}

RobotBase::~RobotBase(void) {}

void RobotBase::rosvelocity_callback(const geometry_msgs::Twist& msg) {
	this->vx_ = msg.linear.x;
	this->vy_ = msg.linear.y;
	this->vz_ = msg.linear.z;
	this->vo_ = msg.angular.z;
}

bool RobotBase::on_set_velocity_(cnbiros_services::SetBaseVelocity::Request& req,
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
