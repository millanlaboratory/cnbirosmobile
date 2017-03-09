#ifndef CNBIROS_CORE_FORCEFIELD_CPP
#define CNBIROS_CORE_FORCEFIELD_CPP

#include "ForceField.hpp"

namespace cnbiros {
	namespace core {

ForceField::ForceField(ros::NodeHandle* node, std::string name) : Navigation(node, name) {

	this->SetParameter("obstruction", 	CNBIROS_FORCEFIELD_OBSTRUCTION, true);
	this->SetParameter("spatialdecay", 	CNBIROS_FORCEFIELD_SPATIALDECAY, true);

	// GridMap Layer initialization
	this->SetGridLayer(CNBIROS_FORCEFIELD_GRIDLAYER);
	
	// GridMap initialization
	this->rosgrid_.add(this->GetGridLayer());
	GridMapTool::SetGeometry(this->rosgrid_, CNBIROS_GRIDMAP_XSIZE, 
							 CNBIROS_GRIDMAP_YSIZE, CNBIROS_GRIDMAP_RESOLUTION);
	GridMapTool::SetFrameId(this->rosgrid_, CNBIROS_GRIDMAP_FRAME);
	GridMapTool::Reset(this->rosgrid_, this->GetGridLayer());
}

ForceField::~ForceField(void) {};

void ForceField::SetGridLayer(std::string layer) {
	this->grid_layer_ = layer;
}

std::string ForceField::GetGridLayer(void) {
	return this->grid_layer_;
}

float ForceField::compute_angle_(float x, float y) {
	return -(std::atan2(y, x) - M_PI/2.0f);
}

float ForceField::compute_distance_(float x, float y) {
	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

float ForceField::compute_lambda_(float distance, float beta1, float beta2) {
	return beta1*exp(-(distance/beta2));
}

float ForceField::compute_sigma_(float distance, float obstruction) {
	return std::atan( std::tan(M_PI/360.0f) + obstruction/(obstruction + distance));
}

bool ForceField::AngularVelocity(geometry_msgs::Twist& msg) {
	
	grid_map::Position 	cPosition;
	grid_map::Index 	cIndex;
	float x, y, angle, distance, strength;
	float lambda, sigma;
	float obstruction, spatialdecay;
	float fobs = 0.0f;

	this->GetParameter("obstruction",  obstruction);
	this->GetParameter("spatialdecay", spatialdecay);


	if(GridMapTool::Exists(this->rosgrid_, this->grid_layer_) == false)
		return false;

	grid_map::Matrix& data = this->rosgrid_[this->grid_layer_];	

	for(grid_map::GridMapIterator it(this->rosgrid_); !it.isPastEnd(); ++it) {

		cIndex = grid_map::Index(*it);

		// get current position
		this->rosgrid_.getPosition(cIndex, cPosition);

		// exclude the positions on the back
		if(cPosition.x() < 0.0f)
			continue;

		// get x and y cohordinates (reverse for standard usage)
		x = -cPosition.y();
		y =  cPosition.x();
		strength = data(cIndex(0), cIndex(1));

		// compute angular velocity based on attractors/repellors
		angle    = this->compute_angle_(x, y);
		distance = this->compute_distance_(x, y);
		lambda   = this->compute_lambda_(distance, strength, spatialdecay);
		sigma    = this->compute_sigma_(distance, obstruction);

		fobs  += lambda*(angle)*exp(-pow((angle),2)/(2.0f*pow(sigma, 2)));

	}
	msg.angular.x = 0.0f;
	msg.angular.y = 0.0f;
	msg.angular.z = fobs;
}

void ForceField::LinearVelocity(geometry_msgs::Twist& msg) {
	msg.linear.x = 0.0f;
	msg.linear.y = 0.0f;
	msg.linear.z = 0.0f;
}

void ForceField::onRunning(void) {
	
	geometry_msgs::Twist msg;
	
	// Compute angular velocity based on force fields
	this->AngularVelocity(msg);

	// Compute linear velocity based on force fields
	this->LinearVelocity(msg);

	// Publish velocity message
	this->Publish(msg);
}

	}
}

#endif
