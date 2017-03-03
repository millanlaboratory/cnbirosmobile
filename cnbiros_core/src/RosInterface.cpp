#ifndef CNBIROS_CORE_ROSINTERFACE_CPP
#define CNBIROS_CORE_ROSINTERFACE_CPP

#include "RosInterface.hpp"

namespace cnbiros {
	namespace core {

RosInterface::RosInterface(void) {
	this->rosnode_ 	  = nullptr;
	this->name_    	  = "rosinterface";
	this->is_stopped_ = false;
	this->SetFrequency(CNBIROS_NODE_FREQUENCY);
}

RosInterface::~RosInterface(void) {}

void RosInterface::Register(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

bool RosInterface::IsRegistered(void) {
	bool isregistered = false;

	if (this->rosnode_ != nullptr)
		isregistered = true;
	
	return isregistered;
}

void RosInterface::SetName(std::string name) {
	this->name_ = name;
}

std::string RosInterface::GetName(void) {
	return this->name_;
}

void RosInterface::SetFrequency(float frequency) {
	this->frequency_ = frequency;
	this->rosrate_   = new ros::Rate(frequency);
}

float RosInterface::GetFrequency(void) {
	return this->frequency_;
}

ros::Subscriber* RosInterface::GetSubscriber(std::string topic) {
	
	auto it = this->rossubs_.find(topic);
	ros::Subscriber* ptr_sub = nullptr;

	if(it != this->rossubs_.end()) {
		ptr_sub = &it->second;
	} else {
		ROS_ERROR("[%s] - Cannot retrieve subscriber on topic %s: does not exist",
				  this->GetName().c_str(), topic.c_str());
	}

	return ptr_sub;
}

void RosInterface::Stop(void) {
	this->is_stopped_ = true;
}

void RosInterface::Resume(void) {
	this->is_stopped_ = false;
}

bool RosInterface::IsStopped(void) {
	return this->is_stopped_;
}

	}
}


#endif
