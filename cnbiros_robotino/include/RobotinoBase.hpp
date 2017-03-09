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
		RobotinoBase(std::string hostname, ros::NodeHandle* node, std::string name = CNBIROS_ROBOTBASE_NAME);
		~RobotinoBase(void);
	
	private:
		void onRunning(void);

	protected:
		std::string hostname_;
		RobotinoCom* com_;
		rec::robotino::api2::OmniDrive omnidrive_;
};

	}
}

#endif
