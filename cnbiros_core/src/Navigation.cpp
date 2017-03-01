#ifndef CNBIROS_CORE_NAVIGATION_CPP
#define CNBIROS_CORE_NAVIGATION_CPP

#include "Navigation.hpp"

namespace cnbiros {
	namespace core {

Navigation::Navigation(ros::NodeHandle* node) {
	
	this->rosnode_ = node;

	// Default initialization
	this->SetName("navigation");
	this->SetFrequency(CNBIROS_NODE_FREQUENCY);
}

Navigation::~Navigation(void) {};

void Navigation::SetName(std::string name) {
	this->name_ = name;
}

void Navigation::SetFrequency(float frequency) {
	this->frequency_ = frequency;
	this->rosrate_   = new ros::Rate(this->frequency_);
}

void Navigation::GetName(std::string& name) {
	name = this->name_;
}

void Navigation::GetFrequency(float& frequency) {
	frequency = this->frequency_;
}


void Navigation::SetPublisher(std::string topic) {

	this->rospub_ = this->rosnode_->advertise<geometry_msgs::Twist>
								(topic, CNBIROS_MESSAGES_BUFFER);
	ROS_INFO("%s advertises topic %s", this->name_.c_str(), topic.c_str());
}

void Navigation::SetSubscriber(std::string topic, unsigned int type) {

	switch(type) {
		case Navigation::MsgType::AsOdometry:
			this->rossub_odometry_ = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, 
											  				   &Navigation::rosodometry_callback, this);
			ROS_INFO("%s subscribe on topic %s (Odometry)", this->name_.c_str(), topic.c_str());
			break;
		case Navigation::MsgType::AsGridMap:
			this->rossub_gridmap_ = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, 
											  				  &Navigation::rosgridmap_callback, this);
			ROS_INFO("%s subscribe on topic %s (GridMap)", this->name_.c_str(), topic.c_str());
			break;
		default:
			break;
	}
}

void Navigation::rosgridmap_callback(const grid_map_msgs::GridMap& msg) {
	this->rosgridmap_msg_ = msg;
}

void Navigation::rosodometry_callback(const nav_msgs::Odometry& msg) {
	this->rosodometry_msg_ = msg;
}

void Navigation::PublishTwist(void) {
	this->rospub_.publish(this->rostwist_msg_);
}

	}
}

#endif
