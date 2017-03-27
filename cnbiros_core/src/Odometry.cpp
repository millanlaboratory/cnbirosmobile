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
	ROS_INFO("%s has been required to start", this->GetName().c_str());
	this->Reset();
}

void Odometry::onStop(void) {
	ROS_INFO("%s has been required to stop", this->GetName().c_str());
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


	}
}

#endif
