#ifndef ROBOT_CPP
#define ROBOT_CPP

#include "Robot.hpp"

namespace cnbiros {
	namespace core {

Robot::Robot(unsigned int type, std::string name, std::string id) {
	this->type_ 		= type;
	this->name_ 		= name;
	this->identifier_	= id;
	this->rosnode_ 		= nullptr;
	this->rosrate_ 	 	= nullptr;
	this->frequency_ 	= CNBIROS_ROBOT_NODE_FREQUENCY;
}

Robot::~Robot(void) {}

unsigned int Robot::GetType(void) {
	return this->type_;
}

void Robot::SetType(unsigned int type) {
	this->type_ = type;
}

std::string Robot::GetName(void) {
	return this->name_;
}

void Robot::SetName(std::string name) {
	this->name_ = name;
}

std::string Robot::GetIdentifier(void) {
	return this->identifier_;
}

void Robot::SetIdentifier(std::string id) {
	this->identifier_ = id;
}

void Robot::Register(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

bool Robot::IsRegistered(void) {
	if(this->rosnode_ != nullptr)
		return true;
	else
		return false;
}

void Robot::Subscribe(std::string topic) {
	if(this->IsRegistered())
		this->rossub_ = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, &Robot::velocityCallback, this);
	else
		ROS_ERROR("Can't subscribe: object is not registered to any node");
	
}


void Robot::Dump(void) {
	printf("[cnbiros] + Robot information:\n");
	printf("          |- Type:\t%d\n", this->GetType());
	printf("          |- Name:\t%s\n", this->GetName().c_str());
	printf("          |- Id:\t%s\n", this->GetIdentifier().c_str());

}

	}
}

#endif
