#ifndef CNBIROS_ROBOTINO_ODOMETRY_HPP
#define CNBIROS_ROBOTINO_ODOMETRY_HPP

#include <rec/robotino/api2/Odometry.h>
#include "Odometry.hpp"
#include "RobotinoCom.hpp"

namespace cnbiros {
	namespace robotino {

class RobotinoOdometry : public cnbiros::core::Odometry, public rec::robotino::api2::Odometry {
	public:
		RobotinoOdometry(std::string hostname,
						 std::string name = CNBIROS_ODOMETRY_NAME);
		~RobotinoOdometry(void);

		void onRunning(void);
		void onReset(void);
	private:
		void readingsEvent(double x, double y, double omega, 
				           float vx, float vy, float vomega, 
						   unsigned int sequence);


	private:
		RobotinoCom* com_;
		std::string  hostname_;

};

	}
}

#endif
