#ifndef ROBOTINO_CPP
#define ROBOTINO_CPP

#include "RobotinoBase.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoBase:: RobotinoBase(std::string hostname, 
					float frequency) :
					core::RobotBase(core::RobotBase::Type::Robotino) {

	// Initialize ros parameters
	this->frequency_ = frequency; 
	this->rosrate_ 	 = new ros::Rate(frequency);
	
	// Initialize motor velocities
	this->vx_ = 0.0f;
	this->vy_ = 0.0f;
	this->vo_ = 0.0f;

	// Initialize connection to the base
	this->com_ = new RobotinoCom("robotino");

	ROS_INFO("RobotinoBase tries to connect to %s...", hostname.c_str());
	this->com_->Connect(hostname);

	// If connected associate omnidrive to comid
	this->omnidrive_.setComId(this->com_->id());
}

RobotinoBase::~RobotinoBase(void) {
	this->com_->Disconnect();
}

bool RobotinoBase::IsConnected(void) {
	return this->com_->IsConnected();
}

void RobotinoBase::AdvertiseOdometry(std::string topic) {

	this->rostopic_odometry_ = topic;
	if(this->IsRegistered()) {
		this->rospub_odometry_ = this->rosnode_->advertise<cnbiros_messages::RobotOdometry>(this->rostopic_odometry_, CNBIROS_MESSAGES_BUFFER);
	} else {
		ROS_ERROR("Can't advertise on %s: object is not registered to any node", topic.c_str());
	}


}

void RobotinoBase::velocityCallback(const geometry_msgs::Twist& msg) {

	//this->vx_ = msg.vx;
	//this->vy_ = msg.vy;
	//this->vo_ = msg.vo;
	//ROS_INFO("New velocity requested: vx=%f, vy=%f, vo=%f", this->vx_, this->vy_, this->vo_);
}

void RobotinoBase::Run(void) {
	
	while(this->rosnode_->ok()) {

		this->omnidrive_.setVelocity(this->vx_, this->vy_, this->vo_);	
		this->com_->processEvents();
		this->rosrate_->sleep();
		
		ros::spinOnce();
	}
}

	}
}

#endif
