#ifndef CNBIROS_CORE_KINECTSCAN_CPP
#define CNBIROS_CORE_KINECTSCAN_CPP

#include "KinectScan.hpp"

namespace cnbiros {
	namespace core {

KinectScan::KinectScan(std::string name) : Sensor(name) {

	// Initialization kinect subscriber
	this->SetSubscriber(CNBIROS_KINECTSCAN_TOPIC, &KinectScan::roskinect_callback_, this);
	
	//// Initialization TF
	//this->SetParentFrame("base_link");
	//this->SetChildFrame("base_kinectscan");

}

KinectScan::~KinectScan(void) {};

void KinectScan::roskinect_callback_(const sensor_msgs::LaserScan& msg) {

	sensor_msgs::LaserScan cmsg = msg;

	this->rosgrid_.Reset(this->GetName());
	this->rosgrid_.Update(this->GetName(), cmsg);

	//float x, y, angle;
	//float maxangle, minangle, maxrange, minrange, angleinc, size;


	//angleinc = msg.angle_increment;
	//maxangle = msg.angle_max;
	//minangle = msg.angle_min;
	//angle 	 = minangle - angleinc;
	//maxrange = msg.range_max;
	//minrange = msg.range_min;
	//size  	 = (maxangle - minangle)/angleinc;


	//this->rosgrid_.Reset(this->sensor_layer_);
	//
	//for (auto i=0; i<size; i++) {
	//	// Update angle for next iteration
	//	angle += angleinc;
	//
	//	// Skip ranges with inf value
	//	if(std::isinf(msg.ranges[i]))
	//		continue;

	//	// Get cartesian cohordinates -> To be added: position of the kinect
	//	x = msg.ranges[i]*cos(angle);
	//	y = msg.ranges[i]*sin(angle);

	//	// Convert x,y cohordinates in position
	//	grid_map::Position position(x, y);
	//	
	//	// Skip positions outside the grid range
	//	if(this->rosgrid_.isInside(position) == false)
	//		continue;

	//	// Fill the grid cell if ranges are between the min/max limits
	//	if (msg.ranges[i] > minrange || msg.ranges[i] < maxrange) {
	//		this->rosgrid_.atPosition(this->sensor_layer_, position) = 1.0f;
	//	}

	//}
}


void KinectScan::onRunning(void) {

	grid_map_msgs::GridMap 	msg;

	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);
}

	}
}

#endif
