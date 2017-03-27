#ifndef CNBIROS_CORE_MOTOR_HPP
#define CNBIROS_CORE_MOTOR_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_services/SetTwist.h"
#include "cnbiros_services/Reset.h"

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
		bool on_service_set_(cnbiros_services::SetTwist::Request& req,
							 cnbiros_services::SetTwist::Response& res);
		bool on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res);
	protected:
		std::string 			topic_;
		geometry_msgs::Twist	motor_twist_;

	private:
		ros::ServiceServer 	srv_set_;
		ros::ServiceServer 	srv_reset_;

};

	}
}

#endif
