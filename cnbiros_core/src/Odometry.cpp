#ifndef ODOMETRY_CPP
#define ODOMETRY_CPP

#include "Odometry.hpp"

namespace cnbiros {
	namespace core {

Odometry::Odometry(ros::NodeHandle* node, float frequency,
				   std::string frameid, std::string child_frameid) {
	this->rosnode_ 	 = node;
	this->frequency_ = frequency;
	this->rosrate_ 	 = new ros::Rate(this->frequency_);

	this->frame_id_       = frameid;
	this->child_frame_id_ = child_frameid;

	// Advertise default CNBIROS_TOPIC_ODOMETRY topic
	this->rospub_ = this->rosnode_->advertise<nav_msgs::Odometry>(CNBIROS_TOPIC_ODOMETRY, 
																  CNBIROS_MESSAGES_BUFFER);
}

Odometry::~Odometry(void) {}

void Odometry::compute_tf(float x, float y, float z, float omega) {

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

void Odometry::compute_odometry(float x, float y, float z, 
								float vx, float vy, float vz, 
								float vomega, unsigned int sequence) {

	// Fill odometry message
	this->rosodom_msg_.header.stamp    = ros::Time::now();
	this->rosodom_msg_.header.frame_id = this->frame_id_;
	this->rosodom_msg_.header.seq      = sequence;

	this->rosodom_msg_.pose.pose.position.x  = x;
	this->rosodom_msg_.pose.pose.position.y  = y;
	this->rosodom_msg_.pose.pose.position.z  = z;
	this->rosodom_msg_.pose.pose.orientation = this->rosodom_quat_;

	this->rosodom_msg_.child_frame_id = this->child_frame_id_;
	this->rosodom_msg_.twist.twist.linear.x = vx;
	this->rosodom_msg_.twist.twist.linear.y = vy;
	this->rosodom_msg_.twist.twist.linear.z = vz;
	this->rosodom_msg_.twist.twist.angular.z  = vomega;

	// Publish the message
	this->rospub_.publish(this->rosodom_msg_);
}



	}
}

#endif
