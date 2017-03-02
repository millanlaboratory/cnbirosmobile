#ifndef CNBIROS_CORE_FORCEFIELD_CPP
#define CNBIROS_CORE_FORCEFIELD_CPP

#include "ForceField.hpp"

namespace cnbiros {
	namespace core {

ForceField::ForceField(ros::NodeHandle* node) : Navigation(node) {
	this->SetName("forcefield");

	this->influence_ 	 = CNBIROS_FORCEFIELD_INFLUENCE;
	this->obstruction_ 	 = CNBIROS_FORCEFIELD_OBSTRUCTION;
	this->strength_  	 = CNBIROS_FORCEFIELD_STRENGTH;
	this->spatialdecay_  = CNBIROS_FORCEFIELD_SPATIALDECAY;
	this->rosgrid_layer_ = "";
}

ForceField::~ForceField(void) {};

void ForceField::SetGridLayer(std::string layer) {
	this->rosgrid_layer_ = layer;
}

std::string ForceField::GetGridLayer(void) {
	return this->rosgrid_layer_;
}

void ForceField::SetInfluence(float radius) {
	this->influence_ = radius;
}

void ForceField::SetObstruction(float size) {
	this->obstruction_ = size;
}

void ForceField::SetStrength(float strength) {
	this->strength_ = strength;
}

void ForceField::SetSpatialDecay(float decay) {
	this->spatialdecay_ = decay;
}

float ForceField::GetInfluence(void) {
	return this->influence_;
}

float ForceField::GetObstruction(void) {
	return this->obstruction_;
}

float ForceField::GetStrength(void) {
	return this->strength_;
}

float ForceField::GetSpatialDecay(void) {
	return this->spatialdecay_;
}

float ForceField::ToAngle(float x, float y) {
	return -(std::atan2(y, x) - M_PI/2.0f);
}

float ForceField::ToRadius(float x, float y) {
	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

float ForceField::GetLambda(float distance, float beta1, float beta2) {
	return beta1*exp(-(distance/beta2));
}

float ForceField::GetSigma(float distance, float obstruction) {
	return std::atan( std::tan(M_PI/360.0f) + obstruction/(obstruction + distance));
}

void ForceField::AngularVelocity(void) {

	grid_map::GridMap grid;
	grid_map::Position current;
	float x, y, angle, radius, lambda, sigma;
	float fobs = 0.0f;
	
	// Convert current grid map message to grid map
	grid_map::GridMapRosConverter::fromMessage(this->rosgridmap_msg_, grid);

	// Get circular iterator submap around the robot
	for(grid_map::CircleIterator it(grid, grid_map::Position(0.0f, 0.0f), this->GetInfluence());
		!it.isPastEnd(); ++it) {
	
		// get current position
		grid.getPosition(*it, current);

		// exclude the positions on the back
		if(current.x() < 0.0f)
			continue;

		// check for the existence of the target layer
		if(grid.exists(this->GetGridLayer()) == false)
			continue;

		// get x and y cohordinates (reverse for standard usage)
		x = -current.y();
		y = current.x();

		// compute angular velocity based on attractors/repellors
		if (grid.at(this->GetGridLayer(), *it) != 0.0f) {

			angle  = this->ToAngle(x, y);
			radius = this->ToRadius(x, y);
			lambda = this->GetLambda(radius, this->GetStrength(), this->GetSpatialDecay());
			sigma  = this->GetSigma(radius, this->GetObstruction());

			fobs  += lambda*(-angle)*exp(-pow((-angle),2)/(2.0f*pow(sigma, 2)));
		}
	}

	this->rostwist_msg_.angular.x = 0.0f;
	this->rostwist_msg_.angular.y = 0.0f;
	this->rostwist_msg_.angular.z = -fobs;
}

void ForceField::LinearVelocity(void) {
	this->rostwist_msg_.linear.x = 0.1f;
	this->rostwist_msg_.linear.y = 0.0f;
	this->rostwist_msg_.linear.z = 0.0f;
}

void ForceField::Run(void) {


	while(this->rosnode_->ok()) {

		// Compute angular velocity based on force fields
		this->AngularVelocity();
		
		// Compute linear velocity based on force fields
		this->LinearVelocity();

		// Publish velocity message
		this->PublishTwist();
		
		ros::spinOnce();
		this->rosrate_->sleep();
	}

}



	}
}

#endif
