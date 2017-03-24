#ifndef CNBIROS_ROBOTINO_COMMUNICATION_CPP
#define CNBIROS_ROBOTINO_COMMUNICATION_CPP

#include "cnbiros_robotino/Communication.hpp"


namespace cnbiros {
	namespace robotino {

Communication::Communication(std::string owner) {
	this->owner_   = owner;
}

Communication::~Communication(void) {
	this->Disconnect();
}

std::string Communication::GetAddress(void) {
	return this->address_;
}

void Communication::SetAddress(std::string address) {
	this->address_ = address;
	this->setAddress(address.c_str());
}

void Communication::Connect(std::string address, bool reconnect) {

	// Disconnect if already connected
	this->Disconnect();

	// Set the address
	this->SetAddress(address);

	// Set connection reconnection
	this->setAutoReconnectEnabled(reconnect);

	// Blocking function
	ROS_INFO("robotino %s tries to connected to %s", this->owner_.c_str(), this->GetAddress().c_str());
	this->connectToServer();
}

void Communication::Disconnect(void) {
	if(this->IsConnected()) {
		this->disconnectFromServer();
	}
}

bool Communication::IsConnected(void) {
	return this->isConnected();
}

void Communication::errorEvent(const char* errorString) {
	ROS_ERROR("robotino %s: %s", this->owner_.c_str(), errorString);
}

void Communication::connectedEvent(void) {
	ROS_INFO("robotino %s connected to %s", this->owner_.c_str(), this->GetAddress().c_str());
}

void Communication::connectionClosedEvent(void) {
	ROS_INFO("robotino %s disconnected from %s", this->owner_.c_str(), this->GetAddress().c_str());
}
	}
}

#endif
