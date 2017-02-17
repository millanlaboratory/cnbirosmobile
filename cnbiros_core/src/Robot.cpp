#ifndef ROBOT_CPP
#define ROBOT_CPP

#include "Robot.hpp"

namespace cnbiros {
	namespace core {

Robot::Robot(unsigned int type, std::string name, std::string id) {
	this->type_ = type;
	this->name_ = name;
	this->identifier_ = id;
	//this->isconnected_ = false;
}

unsigned int Robot::GetType(void) {
	return this->type_;
}

std::string Robot::GetName(void) {
	return this->name_;
}

std::string Robot::GetIdentifier(void) {
	return this->identifier_;
}

void Robot::SetName(std::string name) {
	this->name_ = name;
}

void Robot::SetIdentifier(std::string id) {
	this->identifier_ = id;
}

//bool Robot::IsConnected(void) {
//	return this->isconnected_;
//}

void Robot::Subscribe(ros::NodeHandle* node, std::string topic) {
	this->rossub_ = node->subscribe(topic, 1000, &Robot::velocityCallback, this);
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
