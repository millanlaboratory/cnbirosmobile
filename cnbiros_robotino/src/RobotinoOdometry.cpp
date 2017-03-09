#ifndef CNBIROS_ROBOTINO_ODOMETRY_CPP
#define CNBIROS_ROBOTINO_ODOMETRY_CPP

#include "RobotinoOdometry.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoOdometry::RobotinoOdometry(std::string hostname, 
								   ros::NodeHandle* node,
								   std::string name) :
								   cnbiros::core::Odometry(node, name) {

	// Default values
	this->hostname_     = hostname;

	// Connection to the base
	ROS_INFO("Robotino %s tries to connect to the base (%s)...", 
			 this->GetName().c_str(), this->hostname_.c_str());
	this->com_ = new RobotinoCom(this->GetName());
	this->com_->Connect(this->hostname_);

	// Create odometry association
	this->setComId(this->com_->id());

	// Reset odometry
	this->set(0.0f, 0.0f, 0.0f);
	this->reset_message();

	// Initialize TF
	this->SetParentFrame("odom");
	this->SetChildFrame("base_link");
	
}

RobotinoOdometry::~RobotinoOdometry(void) {}

void RobotinoOdometry::readingsEvent(double x, double y, double omega, 
				           			 float vx, float vy, float vomega, 
						   			 unsigned int sequence) {

	float z  = 0.0f;
	float vz = 0.0f;

	// Compute odometry
	this->set_message(x, y, z, -omega, vx, vy, vz, vomega, sequence);
}

void RobotinoOdometry::onRunning(void) {

	this->com_->processEvents();

	this->Publish(this->rosodom_msg_);
	
	// Transformation
	this->SetTransformMessage(tf::Vector3(this->rosodom_msg_.pose.pose.position.x,
										  this->rosodom_msg_.pose.pose.position.y,
										  this->rosodom_msg_.pose.pose.position.z),
							  			  this->rosodom_msg_.pose.pose.orientation);
}

void RobotinoOdometry::onReset(void) {
	this->set(0.0f, 0.0f, 0.0f);
}

	}
}

#endif
