#ifndef NAVIGATIONCTRL_CPP
#define NAVIGATIONCTRL_CPP

#include "NavigationCtrl.hpp"

namespace cnbiros {
	namespace core {

NavigationCtrl::NavigationCtrl(float frequency) {
	this->rosnode_ 	 = nullptr;
	this->frequency_ = frequency;
	this->rosrate_   = new ros::Rate(frequency);
}

NavigationCtrl::~NavigationCtrl(void) {};

void NavigationCtrl::Register(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

bool NavigationCtrl::IsRegistered(void) {
	if(this->rosnode_ != nullptr) 
		return this->rosnode_->ok();
	else
		return false;
}

void NavigationCtrl::AdvertiseVelocity(std::string topic) {

	this->rostopic_velocity_ = topic;
	if(this->IsRegistered()) {
		this->rospub_velocity_ = this->rosnode_->advertise<cnbiros_messages::RobotVelocity>(this->rostopic_velocity_, CNBIROS_MESSAGES_BUFFER);
	} else {
		ROS_ERROR("Can't advertise on %s: object is not registered to any node", topic.c_str());
	}
}

void NavigationCtrl::SubscribeOdometry(std::string topic) {

	if(this->IsRegistered()) {
		this->rossub_odometry_ = this->rosnode_->subscribe(this->rostopic_odometry_, CNBIROS_MESSAGES_BUFFER, &NavigationCtrl::message_odometry_callback, this);
	} else {
		ROS_ERROR("Can't subscribe to %s: object is not registered to any node", topic.c_str());
	}
}

void NavigationCtrl::SubscribeSensors(std::string topic) {

	if(this->IsRegistered()) {
		this->rossub_sensors_ = this->rosnode_->subscribe(this->rostopic_sensors_, CNBIROS_MESSAGES_BUFFER, &NavigationCtrl::message_sensors_callback, this);
	} else {
		ROS_ERROR("Can't subscribe to %s: object is not registered to any node", topic.c_str());
	}
}

void NavigationCtrl::message_sensors_callback(const grid_map_msgs::GridMap& msg) {

	// Convert grid message to the grid object 
	if(grid_map::GridMapRosConverter::fromMessage(msg, this->grid_) == false)
		ROS_WARN("Cannot convert message to grid");
}

void NavigationCtrl::message_odometry_callback(const cnbiros_messages::RobotOdometry& msg) {

	this->odom_x_ = msg.x;
	this->odom_y_ = msg.y;
	this->odom_z_ = msg.z;
	this->odom_o_ = msg.o;
}

	}
}

#endif
