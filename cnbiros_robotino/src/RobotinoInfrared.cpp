#ifndef ROBOTINOINFRARED_CPP
#define ROBOTINOINFRARED_CPP

#include "RobotinoInfrared.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoInfrared::RobotinoInfrared(std::string hostname, ros::NodeHandle* node) : Sensor(node) {

	// Default values
	this->hostname_     = hostname;
	this->SetName("infrared");

	// Connection to the base
	ROS_INFO("Robotino infrared tries to connect to the base (%s)...", this->hostname_.c_str());
	this->com_ = new RobotinoCom(this->name_);
	this->com_->Connect(this->hostname_);
	
	// Create infrared sensor association
	this->setComId(this->com_->id());
}

RobotinoInfrared::~RobotinoInfrared(void) {}

void RobotinoInfrared::distancesChangedEvent(const float* distances, unsigned int size) {

	float x, y, angle, angleinc, radius;
	float maxdistance, mindistance;

	angleinc    = CNBIROS_ROBOTINO_INFRARED_ANGLE;
	radius 		= CNBIROS_ROBOTINO_RADIUS;
	maxdistance = CNBIROS_ROBOTINO_INFRARED_MAXDISTANCE;
	mindistance = CNBIROS_ROBOTINO_INFRARED_MINDISTANCE;

	this->ResetLayer(this->rosgrid_layer_);
	
	// Iterate along infrared sensors
	for(auto i=0; i<size; i++) {
		
		// Update angle for next iteration
		angle = angleinc*i;
		
		// Get cartesian cohordinates
		x = (distances[i]+radius)*cos(angle);
		y = (distances[i]+radius)*sin(angle);
		
		// Convert x,y cohordinates in position
		grid_map::Position position(x, y);
		
		// Skip positions outside the grid range
		if(this->rosgrid_.isInside(position) == false)
			continue;
		
		// Fill the grid cell if ranges are between the min/max limits
		if (distances[i] > mindistance || distances[i] < maxdistance) {
			this->rosgrid_.atPosition(this->rosgrid_layer_, position) = 1.0f;
		}

		// Fill with 0.0 above the max limit (not down by the API)
		if (distances[i] >= maxdistance)
			this->rosgrid_.atPosition(this->rosgrid_layer_, position) = 0.0f;
		
	}
}

void RobotinoInfrared::Run(void) {

	while(this->rosnode_->ok()) {
	
		// Process robotino infrared events (via api2 callback)
		this->com_->processEvents();
		
		// Publish the grid map	
		this->PublishGrid();

		this->rosrate_->sleep();
		ros::spinOnce();
	}
}

	}
}

#endif
