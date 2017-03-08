#ifndef CNBIROS_CORE_SENSOR_CPP
#define CNBIROS_CORE_SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(ros::NodeHandle* node, std::string name) : RosInterface(node) {

	// Abstract sensor initialization
	this->SetName(name);
	this->SetPublisher<grid_map_msgs::GridMap>("/sensor_" + this->GetName());

	// GridMap initialization
	this->rosgrid_.add(this->GetName());
	this->sensor_layer_ = this->GetName();	
	GridMapTool::SetGeometry(this->rosgrid_, CNBIROS_GRIDMAP_XSIZE, 
							 CNBIROS_GRIDMAP_YSIZE, CNBIROS_GRIDMAP_RESOLUTION);
	GridMapTool::SetFrameId(this->rosgrid_, CNBIROS_GRIDMAP_FRAME);
	GridMapTool::Reset(this->rosgrid_, this->GetName());
}

Sensor::~Sensor(void) {};

void Sensor::onStop(void) {
	grid_map_msgs::GridMap msg;
	
	GridMapTool::Reset(this->rosgrid_);
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(msg);
}

void Sensor::onStart(void) {
	grid_map_msgs::GridMap msg;
	
	GridMapTool::Reset(this->rosgrid_);
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(msg);
}




	}
}



#endif
