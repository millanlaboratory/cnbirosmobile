#ifndef CNBIROS_CORE_FORCEFIELD_CPP
#define CNBIROS_CORE_FORCEFIELD_CPP

#include "ForceField.hpp"

namespace cnbiros {
	namespace core {

ForceField::ForceField(ros::NodeHandle* node) : Navigation(node) {
	this->SetName("forcefield");
}

ForceField::~ForceField(void) {};


void ForceField::AngularVelocity(void) {
}

void ForceField::LinearVelocity(void) {
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
