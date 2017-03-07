#ifndef ROBOTINO_HPP
#define ROBOTINO_HPP

#include <rec/robotino/api2/OmniDrive.h>
#include <rec/robotino/api2/Odometry.h>

#include "RobotBase.hpp"
#include "RobotinoCom.hpp"
#include "cnbiros_messages/RobotOdometry.h"

namespace cnbiros {
	namespace robotino {

class RobotinoBase : public core::RobotBase {
	public:	
		RobotinoBase(std::string hostname, ros::NodeHandle* node);
		~RobotinoBase(void);
		void onRunning(void);

	protected:
		void rosvelocity_callback_(const geometry_msgs::Twist& msg);

	protected:
		std::string hostname_;
		float vx_;
		float vy_;
		float vo_;
		RobotinoCom* com_;
		rec::robotino::api2::OmniDrive omnidrive_;
};

	}
}

#endif
