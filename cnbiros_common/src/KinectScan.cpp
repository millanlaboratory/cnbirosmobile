#ifndef CNBIROS_COMMON_KINECTSCAN_CPP
#define CNBIROS_COMMON_KINECTSCAN_CPP

#include "KinectScan.hpp"

namespace cnbiros {
	namespace common {

KinectScan::KinectScan(std::string name) : Sensor(name) {
	
	float radius;

	// Initialization kinect subscriber
	this->SetSubscriber("/camera/scan", &KinectScan::roskinect_callback_, this);

	// Get parameter from server
	this->GetParameter("radius", radius, 0.0f);
	this->SetRadius(radius);
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
