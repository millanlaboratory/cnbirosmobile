#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string>
#include <ros/ros.h>

#include "cnbiros_messages/SensorMap.h"

#define SENSOR_BUFFER_SIZE 1000

namespace cnbiros {
	namespace core {

class Sensor {

	public:
		Sensor(std::string name);
		virtual ~Sensor(void);
		void Advertise(ros::NodeHandle* node, std::string topic);

		virtual void Read(void) = 0;


	public:
		enum Type {
			Infrared,
			Sonar,
			Kinect,
			Camera
		};

	protected:

		// ros related members
		ros::NodeHandle* rosnode_;
		std::string 	 rostopic_;
		std::string 	 rosname_;
		ros::Publisher 	 rospub_;

};
	}
}

#endif
