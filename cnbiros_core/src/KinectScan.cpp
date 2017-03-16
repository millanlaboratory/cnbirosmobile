#ifndef CNBIROS_CORE_KINECTSCAN_CPP
#define CNBIROS_CORE_KINECTSCAN_CPP

#include "KinectScan.hpp"

namespace cnbiros {
	namespace core {

KinectScan::KinectScan(std::string name) : Sensor(name) {
	
	float radius;
	float rate;

	// Initialization kinect subscriber
	this->SetSubscriber(CNBIROS_KINECTSCAN_TOPIC, &KinectScan::roskinect_callback_, this);

	// Get parameter from server
	this->GetParameter("radius", radius, 0.0f);
	this->GetParameter("rate", rate, CNBIROS_NODE_FREQUENCY);

	this->SetRadius(radius);
	this->SetFrequency(rate);
}

KinectScan::~KinectScan(void) {};

void KinectScan::SetRadius(float radius) {
	this->radius_ = radius;
}

void KinectScan::roskinect_callback_(const sensor_msgs::LaserScan& msg) {

	sensor_msgs::LaserScan cmsg = msg;

	this->rosgrid_.Reset(this->GetName());
	this->rosgrid_.Update(this->GetName(), cmsg, this->radius_);
}


void KinectScan::onRunning(void) {

	grid_map_msgs::GridMap 	msg;

	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);
}

	}
}

#endif
