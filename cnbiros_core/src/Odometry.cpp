#ifndef CNBIROS_CORE_ODOMETRY_CPP
#define CNBIROS_CORE_ODOMETRY_CPP

#include "Odometry.hpp"

namespace cnbiros {
	namespace core {

Odometry::Odometry(ros::NodeHandle* node, std::string name) : RosInterface(node) {

	// Abstract sensor initialization
	this->SetName(name);
	this->SetPublisher<nav_msgs::Odometry>("/" + this->GetName());
	
	// Service for sensor gridmap reset
	this->rossrv_reset_ = this->rosnode_->advertiseService("odometry_reset", 
											&Odometry::on_odometry_reset_, this);
	
	// Initialize TF
	this->SetParentFrame(this->GetName());
	this->SetChildFrame("base_link");
}

Odometry::~Odometry(void) {}

void Odometry::set_message(float x, float y, float z, float omega, 
						   float vx, float vy, float vz, float vomega, 
						   unsigned int sequence) {

	// Fill odometry message
	this->rosodom_msg_.header.stamp    = ros::Time::now();
	this->rosodom_msg_.header.frame_id = this->GetParentFrame();
	this->rosodom_msg_.header.seq      = sequence;

	this->rosodom_msg_.pose.pose.position.x  = x;
	this->rosodom_msg_.pose.pose.position.y  = y;
	this->rosodom_msg_.pose.pose.position.z  = z;
	this->rosodom_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(omega);

	this->rosodom_msg_.child_frame_id = this->GetChildFrame();
	this->rosodom_msg_.twist.twist.linear.x = vx;
	this->rosodom_msg_.twist.twist.linear.y = vy;
	this->rosodom_msg_.twist.twist.linear.z = vz;
	this->rosodom_msg_.twist.twist.angular.z  = vomega;
}

void Odometry::reset_message(void) {

	// Fill odometry message
	this->rosodom_msg_.header.stamp    = ros::Time::now();
	this->rosodom_msg_.header.frame_id = this->GetParentFrame();
	this->rosodom_msg_.header.seq      = 0;

	this->rosodom_msg_.pose.pose.position.x  = 0.0f;
	this->rosodom_msg_.pose.pose.position.y  = 0.0f;
	this->rosodom_msg_.pose.pose.position.z  = 0.0f;
	this->rosodom_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0f);

	//this->rosodom_msg_.child_frame_id = this->child_frame_id_;
	this->rosodom_msg_.twist.twist.linear.x = 0.0f;
	this->rosodom_msg_.twist.twist.linear.y = 0.0f;
	this->rosodom_msg_.twist.twist.linear.z = 0.0f;
	this->rosodom_msg_.twist.twist.angular.z  = 0.0f;
}

bool Odometry::on_odometry_reset_(cnbiros_services::OdometryReset::Request& req,
								  cnbiros_services::OdometryReset::Response& res) {

	res.result = true;

	if(req.reset == true) {
		ROS_INFO("%s requested to reset", this->GetName().c_str());

		this->onReset();
		this->reset_message();
	}

	return res.result;
}

	}
}

#endif
