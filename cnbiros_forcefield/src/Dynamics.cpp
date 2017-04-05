#ifndef CNBIROS_FORCEFIELD_DYNAMIC_CPP
#define CNBIROS_FORCEFIELD_DYNAMIC_CPP

#include "cnbiros_forcefield/Dynamics.hpp"

namespace cnbiros {
	namespace forcefield {

Dynamics::Dynamics(std::string name) : core::Navigation(name) {

	float obstruction, spatialdecay;
	std::string gridlayer;

	this->SetParameter("obstruction",  0.4f,  true);
	this->SetParameter("spatialdecay", 0.5f, true);
	this->layer_ = "actors";

	// SensorGrid initialization
	this->grid_.SetGeometry(CNBIROS_SENSORGRID_X, CNBIROS_SENSORGRID_Y, 
							   CNBIROS_SENSORGRID_R);
	this->grid_.SetFrame(this->GetFrame());
	this->grid_.Reset();
}

Dynamics::~Dynamics(void) {};

float Dynamics::compute_angle_(float x, float y) {
	return -(std::atan2(y, x) - M_PI/2.0f);
}

float Dynamics::compute_distance_(float x, float y) {
	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

float Dynamics::compute_lambda_(float distance, float beta1, float beta2) {
	return beta1*exp(-(distance/beta2));
}

float Dynamics::compute_sigma_(float distance, float obstruction) {
	return std::atan( std::tan(M_PI/360.0f) + obstruction/(obstruction + distance));
}

bool Dynamics::AngularVelocity(geometry_msgs::Twist& msg) {
	
	grid_map::Position 	cPosition;
	grid_map::Index 	cIndex;
	float x, y, angle, distance, strength;
	float lambda, sigma;
	float obstruction, spatialdecay;
	float fobs = 0.0f;

	this->GetParameter("obstruction",  obstruction);
	this->GetParameter("spatialdecay", spatialdecay);

	if(this->grid_.Exists(this->layer_) == false)
		return false;

	grid_map::Matrix& data = this->grid_[this->layer_];	

	for(grid_map::GridMapIterator it(this->grid_); !it.isPastEnd(); ++it) {

		cIndex = grid_map::Index(*it);

		if(data(cIndex(0), cIndex(1)) == 0)
			continue;

		// get current position
		this->grid_.getPosition(cIndex, cPosition);

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

void Dynamics::LinearVelocity(geometry_msgs::Twist& msg) {
	msg.linear.x = 0.0f;
	msg.linear.y = 0.0f;
	msg.linear.z = 0.0f;
}

void Dynamics::onRunning(void) {
	
	geometry_msgs::Twist msg;
	
	// Compute angular velocity based on force fields
	this->AngularVelocity(msg);

	// Compute linear velocity based on force fields
	this->LinearVelocity(msg);

	// Publish velocity message
	this->Publish(this->topic_, msg);
}

	}
}

#endif
