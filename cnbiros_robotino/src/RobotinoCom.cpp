#ifndef ROBOTINOCOM_CPP
#define ROBOTINOCOM_CPP

#include "RobotinoCom.hpp"


namespace cnbiros {
	namespace robotino {

RobotinoCom::RobotinoCom(std::string owner) {
	this->owner_ = owner;
}

void RobotinoCom::Connect(std::string hostname) {

	// Set connection address
	this->setAddress(hostname.c_str());
	
	this->setAutoReconnectEnabled(true);

	// Blocking function
	this->connectToServer();
}

void RobotinoCom::Disconnect(void) {
	this->disconnectFromServer();
}

bool RobotinoCom::IsConnected(void) {
	return this->isConnected();
}

void RobotinoCom::errorEvent(const char* errorString) {
	ROS_ERROR("%s", errorString);
}

void RobotinoCom::connectedEvent(void) {
	ROS_INFO("%s connected to the base", this->owner_.c_str());
}

void RobotinoCom::connectionClosedEvent(void) {
	ROS_INFO("%s disconnected from the base", this->owner_.c_str());
}
	}
}

#endif
