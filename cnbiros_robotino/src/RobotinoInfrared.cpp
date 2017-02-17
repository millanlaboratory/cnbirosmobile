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


	// If connected associate infrared to comid
	this->infraredarray_.setComId(this->com_->id());
}

void RobotinoInfrared::Read(void) {
	
	/*** TESTING GRID_MAP ***/
	grid_map_msgs::GridMap msg;
	grid_map::GridMap map({"test"});

	map.setFrameId("map");
	map.setGeometry(grid_map::Length(2.0, 2.0), 0.1);
	map["test"].setConstant(3.0);
	map.at("test", grid_map::Index(0, 0)) = 10.0f;
	grid_map::GridMapRosConverter::toMessage(map, msg);
	this->rospub_.publish(msg);
	
	/***********************/
	this->com_->processEvents();
}

	}
}

#endif
