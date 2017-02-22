#ifndef ROBOTINOINFRARED_HPP
#define ROBOTINOINFRARED_HPP

#include <algorithm>
#include <rec/robotino/api2/DistanceSensorArray.h>

#include "Robotino.hpp"
#include "RobotinoCom.hpp"
#include "Sensor.hpp"

#define CNBIROS_ROBOTINO_RADIUS 				0.2f		// Radius of the base 				[meters]
#define CNBIROS_ROBOTINO_INFRARED_MAXDISTANCE	0.6f 		// Max distance read by the sensors [meters]
#define CNBIROS_ROBOTINO_INFRARED_ANGLE		 	(40.0f*M_PI)/180.0f
															// Angle (in radians) between two sensors

namespace cnbiros {
	namespace robotino {



class RobotinoInfrared : public core::Sensor, public  rec::robotino::api2::DistanceSensorArray {

	public:
		RobotinoInfrared(std::string hostname, float frequency=CNBIROS_SENSOR_NODE_FREQUENCY);
		virtual ~RobotinoInfrared(void) {};
	
		void SetDecayTime(float time);
		void distancesChangedEvent(const float* distances, unsigned int size);

		void Process(void);
	protected:
		void ProcessDecay(void);
		void gaussian(void);

	protected:
		RobotinoCom* com_;

	private:
		float decayrate_;
};


	}
}




#endif
