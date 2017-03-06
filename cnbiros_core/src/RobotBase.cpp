#ifndef ROBOT_CPP
#define ROBOT_CPP

#include "RobotBase.hpp"

namespace cnbiros {
	namespace core {

RobotBase::RobotBase(ros::NodeHandle* node) : RosInterface() {

	// Default initialization
	this->Register(node);
}

RobotBase::~RobotBase(void) {}

void RobotBase::SubscribeTo(std::string topic) {
	this->SetSubscriber(topic, &RobotBase::rosvelocity_callback_, this);
}

	}
}

#endif
