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
		RobotinoBase(std::string hostname, float frequency);
		~RobotinoBase(void);
		void Run (void);

		bool IsConnected(void);

		void AdvertiseOdometry(std::string topic = CNBIROS_TOPIC_ODOMETRY);
		virtual void velocityCallback(const geometry_msgs::Twist& msg);

	protected:
		RobotinoCom* com_;
		float vx_;
		float vy_;
		float vo_;
		rec::robotino::api2::OmniDrive omnidrive_;
		rec::robotino::api2::Odometry odometry_;

		std::string 	rostopic_odometry_;
		ros::Publisher 	rospub_odometry_;
};

	}
}

#endif
