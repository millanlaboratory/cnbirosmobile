#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "Flags.hpp"
#include "RosInterface.hpp"

namespace cnbiros {
	namespace core {

class RobotBase : public RosInterface {
	public:
		RobotBase(ros::NodeHandle* node);
		virtual ~RobotBase(void);

		void SubscribeTo(std::string topic);

	protected:
		virtual void rosvelocity_callback_(const geometry_msgs::Twist& msg) = 0;


};

	}
}

#endif
