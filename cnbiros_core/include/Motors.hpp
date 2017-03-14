#ifndef CNBIROS_CORE_MOTORS_HPP
#define CNBIROS_CORE_MOTORS_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "Flags.hpp"
#include "RosInterface.hpp"
#include "cnbiros_services/SetBaseVelocity.h"

namespace cnbiros {
	namespace core {

class Motors : public RosInterface {
	public:
		Motors(std::string name);
		virtual ~Motors(void);


	protected:
		virtual void rosvelocity_callback(const geometry_msgs::Twist& msg);

	private:
		bool on_set_velocity_(cnbiros_services::SetBaseVelocity::Request& req,
							  cnbiros_services::SetBaseVelocity::Response& res);
	protected:
		float vx_;
		float vy_;
		float vz_;
		float vo_;

	private:
		ros::ServiceServer 	rossrv_velocity_;

};

	}
}

#endif
