#ifndef CNBIROS_CORE_KINECTSCAN_CPP
#define CNBIROS_CORE_KINECTSCAN_CPP

#include "KinectScan.hpp"

namespace cnbiros {
	namespace core {

KinectScan::KinectScan(ros::NodeHandle* node) : Sensor(node) {
	this->SetName("kinectscan");
}

KinectScan::~KinectScan(void) {};

void KinectScan::SubscribeTo(std::string topic) {
	this->SetSubscriber(topic, &KinectScan::roskinect_callback_, this);
}

void KinectScan::roskinect_callback_(const sensor_msgs::LaserScan& msg) {

	float x, y, angle, radius;
	float maxangle, minangle, maxrange, minrange, angleinc, size;
	
	radius   = 0.01f;
	angleinc = msg.angle_increment;
	maxangle = msg.angle_max;
	minangle = msg.angle_min;
	angle 	 = minangle - angleinc;
	maxrange = msg.range_max;
	minrange = msg.range_min;
	size  	 = (maxangle - minangle)/angleinc;

	GridMapTool::Reset(this->rosgrid_, this->rosgrid_layer_);
	
	for (auto i=0; i<size; i++) {
		// Update angle for next iteration
		angle += angleinc;
	
		// Skip ranges with inf value
		if(std::isinf(msg.ranges[i]))
			continue;

		// Get cartesian cohordinates -> To be added: position of the kinect
		x = (msg.ranges[i]+radius)*cos(angle);
		y = (msg.ranges[i]+radius)*sin(angle);
	
		// Convert x,y cohordinates in position
		grid_map::Position position(x, y);
		
		// Skip positions outside the grid range
		if(this->rosgrid_.isInside(position) == false)
			continue;

		// Fill the grid cell if ranges are between the min/max limits
		if (msg.ranges[i] > minrange || msg.ranges[i] < maxrange) {
			this->rosgrid_.atPosition(this->rosgrid_layer_, position) = 1.0f;
		}

	}
}


void KinectScan::Run(void) {
	
	grid_map_msgs::GridMap msg;
	while(this->rosnode_->ok()) {
		
		// Publish the grid map	
		msg = GridMapTool::ToMessage(this->rosgrid_);
		this->Publish(msg);

		ros::spinOnce();
		this->rosrate_->sleep();
	}
}

	}
}

#endif
