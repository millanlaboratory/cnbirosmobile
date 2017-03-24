#ifndef CNBIROS_ROBOTINO_INFRARED_HPP
#define CNBIROS_ROBOTINO_INFRARED_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <rec/robotino/api2/DistanceSensorArray.h>

#include "cnbiros_robotino/Communication.hpp"
#include "cnbiros_core/Sensor.hpp"

#define CNBIROS_ROBOTINO_BASE_RADIUS 			0.2f				// [m]
#define CNBIROS_ROBOTINO_INFRARED_HEIGHT 		0.03f				// [m]
#define CNBIROS_ROBOTINO_INFRARED_RANGE_MAX		0.41f 				// [m]
#define CNBIROS_ROBOTINO_INFRARED_ANGLE_INC		(40.0f*M_PI)/180.0f // [rad] 

namespace cnbiros {
	namespace robotino {

class Infrared : public core::Sensor<sensor_msgs::PointCloud>, 
				 public rec::robotino::api2::DistanceSensorArray {

	public:
		Infrared(std::string hostname, std::string name = "infrared");
		virtual ~Infrared(void);

		void distancesChangedEvent(const float* ranges, unsigned int size);

		void Reset(void);

	private:
		void onRunning(void);

	private:
		Communication* com_;

};


	}
}




#endif
