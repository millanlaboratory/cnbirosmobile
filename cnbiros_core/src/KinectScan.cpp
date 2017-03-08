#ifndef CNBIROS_CORE_KINECTSCAN_CPP
#define CNBIROS_CORE_KINECTSCAN_CPP

#include "KinectScan.hpp"

namespace cnbiros {
	namespace core {

KinectScan::KinectScan(ros::NodeHandle* node, std::string name) : Sensor(node, name) {

	// Initialization kinect subscriber
	this->SetSubscriber(CNBIROS_KINECTSCAN_TOPIC, &KinectScan::roskinect_callback_, this);
	
	// Initialization TF
	this->SetParentFrame("base_link");
	this->SetChildFrame("base_kinectscan");

	this->SetTransformMessage(tf::Vector3(0.12f, 0.0f, 0.46f), 0.0f);
}

KinectScan::~KinectScan(void) {};

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

	geometry_msgs::PointStamped base_kinect, base_link;
	base_kinect.header.frame_id = this->GetChildFrame();

	GridMapTool::Reset(this->rosgrid_, this->sensor_layer_);
	
	for (auto i=0; i<size; i++) {
		// Update angle for next iteration
		angle += angleinc;
	
		// Skip ranges with inf value
		if(std::isinf(msg.ranges[i]))
			continue;

		// Get cartesian cohordinates -> To be added: position of the kinect
		base_kinect.point.x = msg.ranges[i]*cos(angle);
		base_kinect.point.y = msg.ranges[i]*sin(angle);

		// Transform point from kinect frame to base frame
		this->TransformPoint(this->GetParentFrame(), base_kinect, base_link);

		// Convert x,y cohordinates in position
		grid_map::Position position(base_link.point.x, base_link.point.y);
		
		// Skip positions outside the grid range
		if(this->rosgrid_.isInside(position) == false)
			continue;

		// Fill the grid cell if ranges are between the min/max limits
		if (msg.ranges[i] > minrange || msg.ranges[i] < maxrange) {
			this->rosgrid_.atPosition(this->sensor_layer_, position) = 1.0f;
		}

	}
}


void KinectScan::onRunning(void) {
	
	grid_map_msgs::GridMap msg;
	
	// Publish the grid map	
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(msg);
}

	}
}

#endif
