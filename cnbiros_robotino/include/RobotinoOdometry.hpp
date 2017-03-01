#ifndef ROBOTINO_ODOMETRY_HPP
#define ROBOTINO_ODOMETRY_HPP

#include <rec/robotino/api2/Odometry.h>
#include "Odometry.hpp"
#include "RobotinoCom.hpp"

namespace cnbiros {
	namespace robotino {

class RobotinoOdometry : public cnbiros::core::Odometry, public rec::robotino::api2::Odometry {
	public:
		RobotinoOdometry(std::string hostname,
						 ros::NodeHandle* node,
						 float frequency 	 = CNBIROS_NODE_FREQUENCY,
						 std::string frameid = "odom",
						 std::string child_frameid = "base_link");
		~RobotinoOdometry(void);

	

		void Run(void);
	private:
		void readingsEvent(double x, double y, double omega, 
				           float vx, float vy, float vomega, 
						   unsigned int sequence);


	private:
		RobotinoCom* com_;

};

	}
}

#endif
