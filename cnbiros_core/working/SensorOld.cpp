#ifndef CNBIROS_CORE_SENSOR_CPP
#define CNBIROS_CORE_SENSOR_CPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

Sensor::Sensor(std::string name) : RosInterface(name) {

	// Abstract sensor initialization
	this->rostopic_pub_ = "/sensor_" + this->GetName();
	this->SetPublisher<grid_map_msgs::GridMap>(this->rostopic_pub_);
	
	// Service for sensor gridmap reset
	this->rossrv_reset_ = this->advertiseService("sensor_reset", 
											&Sensor::on_service_reset_, this);
	
	// SensorGrid initialization
	this->rosgrid_.SetGeometry(CNBIROS_SENSORGRID_X, CNBIROS_SENSORGRID_Y, 
							   CNBIROS_SENSORGRID_R);
	this->rosgrid_.AddLayer(this->GetName());
	this->rosgrid_.SetFrame(this->GetFrame());
	this->rosgrid_.Reset();
}

Sensor::~Sensor(void) {};

bool Sensor::on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res) {

	grid_map_msgs::GridMap msg;
	
	res.result = false;
	if(req.reset == true) {
		ROS_INFO("GridMap of %s sensor requested to reset", this->GetName().c_str());
		
		if(this->IsStopped() == false) {
			this->rosgrid_.Reset();
			msg = this->rosgrid_.ToMessage();
			this->Publish(this->rostopic_pub_, msg);
			res.result = true;
		}
	}

	return res.result;
}

void Sensor::onStop(void) {
	grid_map_msgs::GridMap msg;
	
	this->rosgrid_.Reset();
	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);
}

void Sensor::onStart(void) {
	grid_map_msgs::GridMap msg;
	
	this->rosgrid_.Reset();
	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);
}




	}
}



#endif
