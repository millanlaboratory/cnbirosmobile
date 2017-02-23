#ifndef FORCEFIELDCTRL_CPP
#define FORCEFIELDCTRL_CPP

#include "ForceFieldCtrl.hpp"

namespace cnbiros {
	namespace core {

ForceFieldCtrl::ForceFieldCtrl(float frequency) : NavigationCtrl(frequency) {}
ForceFieldCtrl::~ForceFieldCtrl(void) {};


void ForceFieldCtrl::AngularVelocity(void) {
}

void ForceFieldCtrl::LinearVelocity(void) {
}

void ForceFieldCtrl::Run(void) {

	cnbiros_messages::RobotVelocity msg;

	while(this->rosnode_->ok()) {

		// Compute angular velocity based on force fields
		this->AngularVelocity();
		
		// Compute linear velocity based on force fields
		this->LinearVelocity();


		// Fill message
		msg.vx = this->vx_;
		msg.vy = this->vy_;
		msg.vz = this->vz_;
		msg.vo = this->vo_;

		// Publish velocity message
		this->rospub_velocity_.publish(msg);
		
		ros::spinOnce();
		this->rosrate_->sleep();
	}

}



	}
}

#endif
