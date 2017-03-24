#ifndef CNBIROS_ROBOTINO_MOTOR_HPP
#define CNBIROS_ROBOTINO_MOTOR_HPP

#include <rec/robotino/api2/OmniDrive.h>
#include <rec/robotino/api2/Odometry.h>

#include "cnbiros_core/Motor.hpp"
#include "cnbiros_robotino/Communication.hpp"

namespace cnbiros {
	namespace robotino {

class Motor : public core::Motor {
	public:	
		Motor(std::string hostname, std::string name = "motor");
		~Motor(void);
		
		void SetVelocity(const geometry_msgs::Twist& twist);
	private:
		void onRunning(void);

	private:
		Communication* com_;
		rec::robotino::api2::OmniDrive omnidrive_;
};

	}
}

#endif
