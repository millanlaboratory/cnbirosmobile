#ifndef CNBIROS_CORE_MOTORS_HPP
#define CNBIROS_CORE_MOTORS_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "Flags.hpp"
#include "RosInterface.hpp"
#include "cnbiros_services/SetTwist.h"

namespace cnbiros {
	namespace core {

class Motors : public RosInterface {
	public:
		Motors(std::string name);
		virtual ~Motors(void);

	protected:
		virtual void rostwist_callback_(const geometry_msgs::Twist& msg);

	private:
		bool on_service_settwist_(cnbiros_services::SetTwist::Request& req,
							      cnbiros_services::SetTwist::Response& res);
	protected:
		geometry_msgs::Twist 	rostwist_msg_;

	private:
		ros::ServiceServer 	rossrv_twist_;

};

	}
}

#endif
