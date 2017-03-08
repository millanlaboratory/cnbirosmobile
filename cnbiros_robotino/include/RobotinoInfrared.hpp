#ifndef ROBOTINOINFRARED_HPP
#define ROBOTINOINFRARED_HPP

#include <algorithm>
#include <rec/robotino/api2/DistanceSensorArray.h>
#include "RobotinoCom.hpp"
#include "Sensor.hpp"

#define CNBIROS_INFRARED_NAME 	"infrared"

#define CNBIROS_ROBOTINO_RADIUS 				0.2f		// Radius of the base 				[meters]
#define CNBIROS_ROBOTINO_INFRARED_NUMBER		9
#define CNBIROS_ROBOTINO_INFRARED_HEIGHT 		0.03f
#define CNBIROS_ROBOTINO_INFRARED_MINDISTANCE	0.04f 		// Min distance read by the sensors [meters]
#define CNBIROS_ROBOTINO_INFRARED_MAXDISTANCE	0.41f 		// Max distance read by the sensors [meters]
#define CNBIROS_ROBOTINO_INFRARED_ANGLE		 	(40.0f*M_PI)/180.0f
															// Angle (in radians) between two sensors

namespace cnbiros {
	namespace robotino {



class RobotinoInfrared : public core::Sensor, public  rec::robotino::api2::DistanceSensorArray {

	public:
		RobotinoInfrared(ros::NodeHandle* node, 
						 std::string hostname, 
						 std::string name = CNBIROS_INFRARED_NAME);
		virtual ~RobotinoInfrared(void);

		void distancesChangedEvent(const float* distances, unsigned int size);

		void onRunning(void);

	private:
		std::string hostname_;
		RobotinoCom* com_;

};


	}
}




#endif
