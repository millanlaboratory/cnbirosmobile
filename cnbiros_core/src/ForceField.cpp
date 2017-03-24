#ifndef CNBIROS_CORE_FORCEFIELD_CPP
#define CNBIROS_CORE_FORCEFIELD_CPP

#include "cnbiros_core/ForceField.hpp"

namespace cnbiros {
	namespace core {

ForceField::ForceField(std::string name) : Navigation(name) {

	float obstruction, spatialdecay;
	std::string gridlayer;

	// Get parameter from server
	this->GetParameter("obstruction",  obstruction,  CNBIROS_FORCEFIELD_OBSTRUCTION);
	this->GetParameter("spatialdecay", spatialdecay, CNBIROS_FORCEFIELD_SPATIALDECAY);
	this->GetParameter("gridlayer",    gridlayer, std::string("fusion"));
	
	this->SetNavigationParameter("obstruction",  obstruction,  true);
	this->SetNavigationParameter("spatialdecay", spatialdecay, true);
	this->gridlayer_ = gridlayer;

	// SensorGrid initialization
	this->rosgrid_.SetGeometry(CNBIROS_SENSORGRID_X, CNBIROS_SENSORGRID_Y, 
							   CNBIROS_SENSORGRID_R);
	//this->rosgrid_.AddLayer(this->gridlayer_);
	this->rosgrid_.SetFrame(this->GetFrame());
	this->rosgrid_.Reset();
}

ForceField::~ForceField(void) {};

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

	this->GetNavigationParameter("obstruction",  obstruction);
	this->GetNavigationParameter("spatialdecay", spatialdecay);

	if(this->rosgrid_.Exists(this->gridlayer_) == false)
		return false;

	grid_map::Matrix& data = this->rosgrid_[this->gridlayer_];	

	for(grid_map::GridMapIterator it(this->rosgrid_); !it.isPastEnd(); ++it) {

		cIndex = grid_map::Index(*it);

		if(data(cIndex(0), cIndex(1)) == 0)
			continue;

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
	this->Publish(this->rostopic_pub_, msg);
}

	}
}

#endif
