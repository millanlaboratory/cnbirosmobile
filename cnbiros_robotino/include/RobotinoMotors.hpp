#ifndef CNBIROS_ROBOTINO_MOTORS_HPP
#define CNBIROS_ROBOTINO_MOTORS_HPP

#include <rec/robotino/api2/OmniDrive.h>
#include <rec/robotino/api2/Odometry.h>

#include "Motors.hpp"
#include "RobotinoCom.hpp"
#include "cnbiros_messages/RobotOdometry.h"

namespace cnbiros {
	namespace robotino {

class RobotinoMotors : public core::Motors{
	public:	
		RobotinoMotors(std::string hostname, std::string name = CNBIROS_ROBOTBASE_NAME);
		~RobotinoMotors(void);
	
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
