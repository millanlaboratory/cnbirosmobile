#ifndef CNBIROS_CORE_SENSOR_CPP
#define CNBIROS_CORE_SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(ros::NodeHandle* node) {

	// Default initialization
	this->Register(node);
}

Sensor::~Sensor(void) {};

void Sensor::AdvertiseOn(std::string topic) {
	this->SetPublisher<grid_map_msgs::GridMap>(topic);
}

void Sensor::SetGrid(std::string layer, float xsize, float ysize, float res,
					 std::string frame) {

	this->rosgrid_.add(layer);
	this->rosgrid_layer_ = layer;	
	GridMapTool::SetGeometry(this->rosgrid_, xsize, ysize, res);
	GridMapTool::SetFrameId(this->rosgrid_, frame);
	GridMapTool::Reset(this->rosgrid_, layer);
}






	}
}



#endif
