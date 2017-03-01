#ifndef ROBOTINO_ODOMETRY_CPP
#define ROBOTINO_ODOMETRY_CPP

#include "RobotinoOdometry.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoOdometry::RobotinoOdometry(std::string hostname,
								   ros::NodeHandle* node,
								   float frequency,
								   std::string frameid,
								   std::string child_frameid) : 
								   cnbiros::core::Odometry(node, frequency, frameid, child_frameid) {

	// Connection to the base
	ROS_INFO("Robotino infrared tries to connect to the base (%s)...", hostname.c_str());
	this->com_ = new RobotinoCom("odometry");
	this->com_->Connect(hostname);

	// Create infrared sensor association
	this->setComId(this->com_->id());
}

RobotinoOdometry::~RobotinoOdometry(void) {}

void RobotinoOdometry::readingsEvent(double x, double y, double omega, 
				           			 float vx, float vy, float vomega, 
						   			 unsigned int sequence) {

	float z  = 0.0f;
	float vz = 0.0f;

	// Compute tf transformation
	this->compute_tf(x, y, z, omega);

	// Compute odometry
	this->compute_odometry(x, y, z, vx, vy, vz, vomega, sequence);
}

void RobotinoOdometry::Run(void) {

	while(this->rosnode_->ok()) {
		
		this->com_->processEvents();
		
		this->rosrate_->sleep();
		ros::spinOnce();

	}
}


	}
}

#endif
