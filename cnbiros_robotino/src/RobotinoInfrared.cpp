#ifndef ROBOTINOINFRARED_CPP
#define ROBOTINOINFRARED_CPP

#include "RobotinoInfrared.hpp"

namespace cnbiros {
	namespace robotino {

RobotinoInfrared::RobotinoInfrared(std::string hostname, 
								   unsigned int frequency,
								   float xdim, float ydim, float res,
								   std::string layer, 
								   std::string frameid) : Sensor("infrared") {

	// Initialize parameters
	this->frequency_ = frequency;
	this->rosrate_ 	 = new ros::Rate(frequency);
	this->SetGridParameters(layer, frameid);
	this->SetGridDimensions(xdim, ydim, res);

	// Connection to the base
	ROS_INFO("Robotino infrared tries to connect to the base (%s)...", hostname.c_str());
	this->com_ = new RobotinoCom("infrared");
	this->com_->Connect(hostname);

	// Configuration of Grid map
	this->grid_.add(this->grid_base_layer_, 0.0f);
	this->grid_.setFrameId(this->grid_base_frameid_);
	this->grid_.setGeometry(grid_map::Length(this->grid_xdim_, 
											 this->grid_ydim_), 
							this->grid_resolution_);
	this->grid_[this->grid_base_layer_].setConstant(0.0);
	this->decaystep_ = 0.0f;

	// Create infrared sensor association
	this->setComId(this->com_->id());
}

void RobotinoInfrared::SetDecayTime(float time) {
	this->decaystep_ = 1.0f/float(this->frequency_ * time);

}

void RobotinoInfrared::distancesChangedEvent(const float* distances, unsigned int size) {

	float x, y;

	// In case the decay is set to 0, clear the grid once the callback is called
	if(this->decaystep_ == 0) {
		this->grid_[this->grid_base_layer_].setConstant(0.0);
	}
	
	// Iterate along infrared sensors
	for(auto i=0; i<size; i++) {
		x = (distances[i]+ROBOTINO_BASE_RADIUS)*cos(ROBOTINO_INFRARED_ANGLE*i);
		y = (distances[i]+ROBOTINO_BASE_RADIUS)*sin(ROBOTINO_INFRARED_ANGLE*i);

		grid_map::Position position(x, y);
		if (distances[i] <= ROBOTINO_INFRARED_MAX_DISTANCE-ROBOTINO_BASE_RADIUS) {
			this->grid_.atPosition(this->grid_base_layer_, position) = 1.0f;
		}
	}
}

void RobotinoInfrared::ProcessDecay(void) {
	
	// Substract the decay step from the grid
	this->grid_[this->grid_base_layer_].array() -= this->decaystep_;

	// Set to 0 every negative value
	for (grid_map::GridMapIterator it(this->grid_); !it.isPastEnd(); ++it) {
		if(this->grid_.at(this->grid_base_layer_, *it) < 0)
			this->grid_.at(this->grid_base_layer_, *it) = 0.0f;
    }
}

void RobotinoInfrared::Process(void) {
	
	while(this->rosnode_->ok()) {
		
		// Process robotino infrared events (via api2 callback)
		this->com_->processEvents();

		// Process map decay
		this->ProcessDecay();
		
		// Publish the msg
		grid_map::GridMapRosConverter::toMessage(this->grid_, this->msg_);
		this->rospub_.publish(this->msg_);

		this->rosrate_->sleep();
		ros::spinOnce();
	}
}

/*
 * Test with gaussian. Working but should be moved at the Fusion level
 *

void RobotinoInfrared::gaussian(void) {

	float xc, yc, x, y;

	this->grid_["proc"].setConstant(0.0);
	for (grid_map::GridMapIterator oit(this->grid_); !oit.isPastEnd(); ++oit) {
		if(this->grid_.at("infrared", *oit) > 0) {
			grid_map::Position ccenter;
			const grid_map::Index oidx(*oit);
			this->grid_.getPosition(oidx, ccenter);

			xc = ccenter(0);
			yc = ccenter(1);
			for(grid_map::CircleIterator iit(this->grid_, ccenter, 0.2);
					!iit.isPastEnd(); ++iit) {
				const grid_map::Index iidx(*iit);
				grid_map::Position cpos;
				this->grid_.getPosition(iidx, cpos);

				x = cpos(0);
				y = cpos(1);
				this->grid_.at("proc", *iit) = std::max(this->grid_.at("proc", *iit), float(this->grid_.at("infrared", *oit))*float(exp(-(pow(x - xc, 2) + pow(y-yc, 2))/(0.01f))));
			}
		}
	}
}
*/

	}
}

#endif
