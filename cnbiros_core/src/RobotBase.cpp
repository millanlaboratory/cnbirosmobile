#ifndef ROBOT_CPP
#define ROBOT_CPP

#include "RobotBase.hpp"

namespace cnbiros {
	namespace core {

RobotBase::RobotBase(unsigned int type, std::string name, std::string id) {
	this->type_ 		= type;
	this->name_ 		= name;
	this->identifier_	= id;
	this->rosnode_ 		= nullptr;
	this->rosrate_ 	 	= nullptr;
	this->frequency_ 	= CNBIROS_ROBOT_NODE_FREQUENCY;
}

RobotBase::~RobotBase(void) {}

unsigned int RobotBase::GetType(void) {
	return this->type_;
}

void RobotBase::SetType(unsigned int type) {
	this->type_ = type;
}

std::string RobotBase::GetName(void) {
	return this->name_;
}

void RobotBase::SetName(std::string name) {
	this->name_ = name;
}

std::string RobotBase::GetIdentifier(void) {
	return this->identifier_;
}

void RobotBase::SetIdentifier(std::string id) {
	this->identifier_ = id;
}

void RobotBase::Register(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

bool RobotBase::IsRegistered(void) {
	if(this->rosnode_ != nullptr)
		return true;
	else
		return false;
}

void RobotBase::Subscribe(std::string topic) {
	if(this->IsRegistered())
		this->rossub_ = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, &RobotBase::velocityCallback, this);
	else
		ROS_ERROR("Can't subscribe: object is not registered to any node");
	
}


void RobotBase::Dump(void) {
	printf("[cnbiros] + RobotBase information:\n");
	printf("          |- Type:\t%d\n", this->GetType());
	printf("          |- Name:\t%s\n", this->GetName().c_str());
	printf("          |- Id:\t%s\n", this->GetIdentifier().c_str());

}

	}
}

#endif
