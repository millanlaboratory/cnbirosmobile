#ifndef ODOMETRY_CPP
#define ODOMETRY_CPP

#include "Odometry.hpp"

namespace cnbiros {
	namespace core {

Odometry::Odometry(ros::NodeHandle* node) : RosInterface(node) {

	// Initialize TF
	this->SetParentFrame("odom");
	this->SetChildFrame("base_link");
}

Odometry::~Odometry(void) {}

void Odometry::AdvertiseOn(std::string topic) {
	this->SetPublisher<nav_msgs::Odometry>(topic);
}


nav_msgs::Odometry Odometry::ConvertToMessage(float x, float y, float z, 
											  float omega, float vx, float vy, 
											  float vz, float vomega, 
											  unsigned int sequence) {

	nav_msgs::Odometry odom_msg;

	// Fill odometry message
	odom_msg.header.stamp    = ros::Time::now();
	odom_msg.header.frame_id = this->GetParentFrame();
	odom_msg.header.seq      = sequence;

	odom_msg.pose.pose.position.x  = x;
	odom_msg.pose.pose.position.y  = y;
	odom_msg.pose.pose.position.z  = z;
	odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(omega);

	odom_msg.child_frame_id = this->GetChildFrame();
	odom_msg.twist.twist.linear.x = vx;
	odom_msg.twist.twist.linear.y = vy;
	odom_msg.twist.twist.linear.z = vz;
	odom_msg.twist.twist.angular.z  = vomega;

	return odom_msg;
}

void Odometry::Reset(nav_msgs::Odometry& msg) {

	// Fill odometry message
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = "odom";
	msg.header.seq      = 0;

	msg.pose.pose.position.x  = 0.0f;
	msg.pose.pose.position.y  = 0.0f;
	msg.pose.pose.position.z  = 0.0f;
	msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0f);

	//this->rosodom_msg_.child_frame_id = this->child_frame_id_;
	msg.twist.twist.linear.x = 0.0f;
	msg.twist.twist.linear.y = 0.0f;
	msg.twist.twist.linear.z = 0.0f;
	msg.twist.twist.angular.z  = 0.0f;
}

/*
void Odometry::ConvertToTffloat x, float y, float z, float omega) {

	// Create quaternion from yaw (angle)
	this->rosodom_quat_ = tf::createQuaternionMsgFromYaw(omega);

	// Fill tf message
	this->rosodom_tf_.header.stamp 	  = ros::Time::now();
	this->rosodom_tf_.header.frame_id = this->frame_id_;
	this->rosodom_tf_.child_frame_id  = this->child_frame_id_;

	this->rosodom_tf_.transform.translation.x = x;
	this->rosodom_tf_.transform.translation.y = y;
	this->rosodom_tf_.transform.translation.z = z;
	this->rosodom_tf_.transform.rotation	  = this->rosodom_quat_;

	// Broadcast the transformation
	this->rostf_.sendTransform(this->rosodom_tf_);
}
*/

	}
}

#endif
