#ifndef CNBIROS_ROBOTINO_ODOMETRY_CPP
#define CNBIROS_ROBOTINO_ODOMETRY_CPP

#include "cnbiros_robotino/Odometry.hpp"

namespace cnbiros {
	namespace robotino {

Odometry::Odometry(std::string name) : cnbiros::core::Odometry(name) {

	// Connection to the robot
	this->com_ = new Communication(this->GetName());
}

Odometry::~Odometry(void) {
	this->com_->Disconnect();
	delete this->com_;
}

bool Odometry::Connect(std::string hostname, bool blocking) {

	do {
		try {
			this->com_->Connect(hostname);
		} catch (rec::robotino::api2::RobotinoException &e) {};

		ros::Duration(1.0f).sleep();
	} while((this->com_->IsConnected() == false) && (blocking == true));

	// If connected associate to comid
	if(this->com_->IsConnected()) {
		this->setComId(this->com_->id());
	}

	return this->com_->IsConnected();
}

void Odometry::SetOdometry(const nav_msgs::Odometry& odom) {
	
	float x, y, phi;
	tf::Pose pose;

	this->odometry_data_ = odom;
	tf::poseMsgToTF(odom.pose.pose, pose);

	x   = odom.pose.pose.position.x;
	y   = odom.pose.pose.position.y;
	phi = tf::getYaw(pose.getRotation());
	
	if(this->set(x, y, phi, true) == false) {
		ROS_WARN("%s cannot set odometry", this->GetName().c_str());
	}
}

void Odometry::readingsEvent(double x, double y, double omega, 
				           	 float vx, float vy, float vomega, 
						   	 unsigned int sequence) {

	float z  = 0.0f;
	float vz = 0.0f;
	nav_msgs::Odometry odom;

	this->odometry_data_.header.stamp    = ros::Time::now();
	this->odometry_data_.header.seq      = sequence;

	this->odometry_data_.pose.pose.position.x  = x;
	this->odometry_data_.pose.pose.position.y  = y;
	this->odometry_data_.pose.pose.position.z  = z;
	this->odometry_data_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(omega);

	this->odometry_data_.twist.twist.linear.x  = vx;
	this->odometry_data_.twist.twist.linear.y  = vy;
	this->odometry_data_.twist.twist.linear.z  = vz;
	this->odometry_data_.twist.twist.angular.z = vomega;

}

void Odometry::onRunning(void) {

	this->com_->processEvents();

	this->Publish(this->topic_, this->odometry_data_);
}

	}
}

#endif
