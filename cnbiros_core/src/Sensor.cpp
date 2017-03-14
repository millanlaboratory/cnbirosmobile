#ifndef CNBIROS_CORE_SENSOR_CPP
#define CNBIROS_CORE_SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(std::string name) {

	// Abstract sensor initialization
	this->SetName(name);
	this->rostopic_pub_ = "/sensor_" + this->GetName();
	this->SetPublisher<grid_map_msgs::GridMap>(this->rostopic_pub_);


	// Service for sensor gridmap reset
	this->rossrv_reset_ = this->advertiseService("sensor_reset", 
											&Sensor::on_service_reset_, this);
	
	// GridMap initialization
	this->rosgrid_.add(this->GetName());
	this->sensor_layer_ = this->GetName();	
	GridMapTool::SetGeometry(this->rosgrid_, CNBIROS_GRIDMAP_XSIZE, 
							 CNBIROS_GRIDMAP_YSIZE, CNBIROS_GRIDMAP_RESOLUTION);
	GridMapTool::SetFrameId(this->rosgrid_, CNBIROS_GRIDMAP_FRAME);
	GridMapTool::Reset(this->rosgrid_, this->GetName());
}

Sensor::~Sensor(void) {};

void Sensor::GetGridMap(grid_map::GridMap* grid) {
	grid = &this->rosgrid_;
}


bool Sensor::on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res) {

	grid_map_msgs::GridMap msg;
	
	res.result = false;
	if(req.reset == true) {
		ROS_INFO("GridMap of %s sensor requested to reset", this->GetName().c_str());
		
		if(this->IsStopped() == false) {
			GridMapTool::Reset(this->rosgrid_);
			GridMapTool::ToMessage(this->rosgrid_, msg);
			this->Publish(this->rostopic_pub_, msg);
			res.result = true;
		}
	}

	return res.result;
}

void Sensor::onStop(void) {
	grid_map_msgs::GridMap msg;
	
	GridMapTool::Reset(this->rosgrid_);
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(this->rostopic_pub_, msg);
}

void Sensor::onStart(void) {
	grid_map_msgs::GridMap msg;
	
	GridMapTool::Reset(this->rosgrid_);
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(this->rostopic_pub_, msg);
}




	}
}



#endif
