#ifndef CNBIROS_ROBOTINO_INFRARED_CPP
#define CNBIROS_ROBOTINO_INFRARED_CPP

#include "RobotinoInfrared.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoInfrared::RobotinoInfrared(std::string hostname, 
								   std::string name) : core::Sensor(name) {

	float radius;
	float rate;

	// Default values
	this->hostname_     = hostname;

	// Connection to the base
	ROS_INFO("Robotino %s tries to connect to the base (%s)...", 
			 this->GetName().c_str(), this->hostname_.c_str());
	this->com_ = new RobotinoCom(this->GetName());
	this->com_->Connect(this->hostname_);
	
	// Create infrared sensor association
	this->setComId(this->com_->id());

	// Get parameter from server
	this->GetParameter("radius", radius, CNBIROS_ROBOTINO_RADIUS);
	this->GetParameter("rate", rate, CNBIROS_NODE_FREQUENCY);

	this->SetRadius(radius);
	this->SetFrequency(rate);
}

RobotinoInfrared::~RobotinoInfrared(void) {}

void RobotinoInfrared::SetRadius(float radius) {
	this->radius_ = radius;
}

void RobotinoInfrared::distancesChangedEvent(const float* distances, unsigned int size) {

	sensor_msgs::LaserScan msg;

	msg.header.frame_id = this->GetFrame();
	msg.angle_min 		= 0.0f;
	msg.angle_max 		= size*CNBIROS_ROBOTINO_INFRARED_ANGLE;
	msg.angle_increment = CNBIROS_ROBOTINO_INFRARED_ANGLE;
	msg.range_min 		= CNBIROS_ROBOTINO_INFRARED_MINDISTANCE;
	msg.range_max 		= CNBIROS_ROBOTINO_INFRARED_MAXDISTANCE;
	msg.ranges    		= std::vector<float>(distances, distances + size);


	this->rosgrid_.Reset(this->GetName());
	this->rosgrid_.Update(this->GetName(), msg, this->radius_);
}

void RobotinoInfrared::onRunning(void) {
	
	grid_map_msgs::GridMap msg;

	// Process robotino infrared events (via api2 callback)
	this->com_->processEvents();

	// Publish the sensor grid	
	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);

}

	}
}

#endif
