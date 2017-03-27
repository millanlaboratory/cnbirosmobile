#ifndef CNBIROS_ROBOTINO_ODOMETRY_HPP
#define CNBIROS_ROBOTINO_ODOMETRY_HPP
#include "tf/transform_datatypes.h"
#include <rec/robotino/api2/Odometry.h>
#include "cnbiros_core/Odometry.hpp"
#include "cnbiros_robotino/Communication.hpp"

namespace cnbiros {
	namespace robotino {

class Odometry : public cnbiros::core::Odometry, public rec::robotino::api2::Odometry {
	
	public:
		Odometry(std::string hostname, std::string name = "odometry");
		~Odometry(void);
		

		void SetOdometry(const nav_msgs::Odometry& odom);
	
	private:
		void readingsEvent(double x, double y, double omega, 
				           float vx, float vy, float vomega, 
						   unsigned int sequence);

		void onRunning(void);

	private:
		Communication* com_;


};

	}
}

#endif
