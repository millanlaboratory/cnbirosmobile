#ifndef CNBIROS_CORE_MOTOR_HPP
#define CNBIROS_CORE_MOTOR_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "cnbiros_core/Flags.hpp"
#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_services/SetTwist.h"

namespace cnbiros {
	namespace core {

class Motor : public RosInterface {
	public:
		Motor(std::string name);
		virtual ~Motor(void);
	
		void GetVelocity(geometry_msgs::Twist& twist);
		virtual void SetVelocity(const geometry_msgs::Twist& twist) = 0;

		void Reset(void);
	protected:
		void onReceived(const geometry_msgs::Twist& msg);
		virtual void onStop(void);
		virtual void onStart(void);
		virtual void onRunning(void) {};

	private:
		bool on_service_settwist_(cnbiros_services::SetTwist::Request& req,
							      cnbiros_services::SetTwist::Response& res);
	protected:
		geometry_msgs::Twist	motor_twist_;

	private:
		ros::ServiceServer 	srv_twist_;

};

	}
}

#endif
