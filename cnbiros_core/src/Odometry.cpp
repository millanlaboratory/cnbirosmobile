#ifndef CNBIROS_CORE_ODOMETRY_CPP
#define CNBIROS_CORE_ODOMETRY_CPP

#include "cnbiros_core/Odometry.hpp"

namespace cnbiros {
	namespace core {

Odometry::Odometry(std::string name) : RosInterface(name) {

	// Abstract sensor initialization
	this->topic_ = "/odom";
	this->SetPublisher<nav_msgs::Odometry>(this->topic_);
	
	// Service for sensor gridmap reset
	this->srv_set_   = this->advertiseService("set_odometry", &Odometry::on_service_set_, this);
	this->srv_reset_ = this->advertiseService("reset_odometry", &Odometry::on_service_reset_, this);
}

Odometry::~Odometry(void) {}

void Odometry::GetOdometry(nav_msgs::Odometry& odom) {
	odom = odometry_data_;
}

void Odometry::Reset(void) {

	this->odometry_data_.pose.pose.position.x  = 0.0f;
	this->odometry_data_.pose.pose.position.y  = 0.0f;
	this->odometry_data_.pose.pose.position.z  = 0.0f;
	this->odometry_data_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0f);
	this->odometry_data_.twist.twist.linear.x = 0.0f;
	this->odometry_data_.twist.twist.linear.y = 0.0f;
	this->odometry_data_.twist.twist.linear.z = 0.0f;
	this->odometry_data_.twist.twist.angular.z  = 0.0f;

	this->SetOdometry(this->odometry_data_);
}

void Odometry::onStart(void) {
	this->Reset();
}

void Odometry::onStop(void) {
	this->Reset();
}


bool Odometry::on_service_set_(cnbiros_services::SetOdometry::Request& req,
							   cnbiros_services::SetOdometry::Response& res) {

	res.result = true;
	this->SetOdometry(req.odometry);
	return res.result;
}

bool Odometry::on_service_reset_(cnbiros_services::Reset::Request& req,
								 cnbiros_services::Reset::Response& res) {

	res.result = true;
	this->Reset();
	this->Publish(this->topic_, this->odometry_data_);

	return res.result;
}


/*
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
*/


	}
}

#endif
