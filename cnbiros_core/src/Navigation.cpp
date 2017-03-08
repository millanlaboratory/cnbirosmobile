#ifndef CNBIROS_CORE_NAVIGATION_CPP
#define CNBIROS_CORE_NAVIGATION_CPP

#include "Navigation.hpp"

namespace cnbiros {
	namespace core {

Navigation::Navigation(ros::NodeHandle* node) : RosInterface(node) {

	// Default initialization
	this->SetName("navigation");
	this->has_message_ = false;
}

Navigation::~Navigation(void) {};

void Navigation::SubscribeTo(std::string topic) {
	this->SetSubscriber(topic, &Navigation::rosgridmap_callback_, this);
}

void Navigation::AdvertiseOn(std::string topic) {
	this->SetPublisher<geometry_msgs::Twist>(topic);
}

void Navigation::rosgridmap_callback_(const grid_map_msgs::GridMap& msg) {
	this->has_message_ = true;
	this->rosgridmap_msg_ = msg;
}


	}
}

#endif
