#ifndef ROBOTINOINFRARED_CPP
#define ROBOTINOINFRARED_CPP

#include "RobotinoInfrared.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoInfrared::RobotinoInfrared(std::string hostname, 
								   std::string name) : Sensor(name) {

	// Initialize connection to the base
	this->com_ = new RobotinoCom(name);

	ROS_INFO("%s tries to connect to %s...", name.c_str(), hostname.c_str());
	this->com_->Connect(hostname);

	// Create and configure Grid map
	this->grid_.add("infrared");
	this->grid_.setFrameId("robot");
	this->grid_.setGeometry(grid_map::Length(2.0, 2.0), 0.05);
	this->grid_["infrared"].setConstant(0.0);

	// Create infrared sensor association
	this->setComId(this->com_->id());
}

void RobotinoInfrared::distancesChangedEvent(const float* distances, unsigned int size) {

	float x, y;
	this->grid_["infrared"].setConstant(0.0);
	// Iterate along infrared sensors
	for(auto i = 0; i < size; ++i) {
		
		// 0.698 radians = 40 Degrees
		// 0.2 is the radius of the robot
		x = ( distances[i] + 0.2 ) * cos(0.698 * i);
		y = ( distances[i] + 0.2 ) * sin(0.698 * i);
		grid_map::Position position(x, y);
		printf("[%d] - x=%f, y=%f, Distance: %f\n", i, x, y, distances[i] + 0.2);  
		if (distances[i] < 0.4) {
			this->grid_.atPosition("infrared", position) = 1.0f;
		}

	}


	grid_map::GridMapRosConverter::toMessage(this->grid_, this->msg_);

	// Publish the msg
	this->rospub_.publish(this->msg_);
}


void RobotinoInfrared::Read(void) {
	
	/*** TESTING GRID_MAP ***/

	//map.setFrameId("map");
	//map.setGeometry(grid_map::Length(2.0, 2.0), 0.1);
	//map["test"].setConstant(3.0);
	//map.at("test", grid_map::Index(0, 0)) = 10.0f;
	//grid_map::GridMapRosConverter::toMessage(map, msg);
	//this->rospub_.publish(msg);
	
	/***********************/
	this->com_->processEvents();
}

	}
}

#endif
