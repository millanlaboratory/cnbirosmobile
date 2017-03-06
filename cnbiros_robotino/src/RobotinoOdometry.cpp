#ifndef ROBOTINO_ODOMETRY_CPP
#define ROBOTINO_ODOMETRY_CPP

#include "RobotinoOdometry.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoOdometry::RobotinoOdometry(std::string hostname, ros::NodeHandle* node) :
								   cnbiros::core::Odometry(node) {

	// Default values
	this->hostname_     = hostname;
	this->SetName("odometry");

	// Connection to the base
	ROS_INFO("Robotino %s tries to connect to the base (%s)...", 
			 this->GetName().c_str(), this->hostname_.c_str());
	this->com_ = new RobotinoCom(this->GetName());
	this->com_->Connect(this->hostname_);

	// Create odometry association
	this->setComId(this->com_->id());
}

RobotinoOdometry::~RobotinoOdometry(void) {}

void RobotinoOdometry::readingsEvent(double x, double y, double omega, 
				           			 float vx, float vy, float vomega, 
						   			 unsigned int sequence) {

	float z  = 0.0f;
	float vz = 0.0f;

	// Compute odometry
	this->compute_odometry(x, y, z, vx, vy, vz, vomega, sequence);
}

void RobotinoOdometry::Run(void) {

	while(this->rosnode_->ok()) {
		
	
		this->com_->processEvents();
		this->Publish(this->rosodom_msg_);
		
		this->rosrate_->sleep();
		ros::spinOnce();

	}
}


	}
}

#endif
