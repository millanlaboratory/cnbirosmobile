#ifndef CNBIROS_CORE_KINECTSCAN_CPP
#define CNBIROS_CORE_KINECTSCAN_CPP

#include "KinectScan.hpp"

namespace cnbiros {
	namespace core {

KinectScan::KinectScan(std::string name) : Sensor(name) {
	
	float radius = 0.0f;

	// Initialization kinect subscriber
	this->SetSubscriber(CNBIROS_KINECTSCAN_TOPIC, &KinectScan::roskinect_callback_, this);

	// Get radius parameter from parameter server (if exists)
	if(this->getParam("radius", radius)) {
		ROS_INFO("Retrieved radius parameter for %s: %f", this->GetName().c_str(), radius);
	} else {
		ROS_WARN("Radius parameter not found for %s. Using default: 0.0f", this->GetName().c_str());
	}

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
