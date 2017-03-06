#ifndef ODOMETRY_CPP
#define ODOMETRY_CPP

#include "Odometry.hpp"

namespace cnbiros {
	namespace core {

Odometry::Odometry(ros::NodeHandle* node) {

	this->Register(node);
}

Odometry::~Odometry(void) {}

void Odometry::AdvertiseOn(std::string topic) {
	this->SetPublisher<nav_msgs::Odometry>(topic);
}


void Odometry::compute_odometry(float x, float y, float z, 
								float vx, float vy, float vz, 
								float vomega, unsigned int sequence) {

	// Fill odometry message
	this->rosodom_msg_.header.stamp    = ros::Time::now();
	this->rosodom_msg_.header.frame_id = "odom";
	this->rosodom_msg_.header.seq      = sequence;

	this->rosodom_msg_.pose.pose.position.x  = x;
	this->rosodom_msg_.pose.pose.position.y  = y;
	this->rosodom_msg_.pose.pose.position.z  = z;
	//this->rosodom_msg_.pose.pose.orientation = this->rosodom_quat_;

	//this->rosodom_msg_.child_frame_id = this->child_frame_id_;
	this->rosodom_msg_.twist.twist.linear.x = vx;
	this->rosodom_msg_.twist.twist.linear.y = vy;
	this->rosodom_msg_.twist.twist.linear.z = vz;
	this->rosodom_msg_.twist.twist.angular.z  = vomega;
}


/* To be moved to separeted object 
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
*/

	}
}

#endif
