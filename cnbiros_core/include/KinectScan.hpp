#ifndef CNBIROS_CORE_KINECTSCAN_HPP
#define CNBIROS_CORE_KINECTSCAN_HPP

#include <sensor_msgs/LaserScan.h>
#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

class KinectScan : public Sensor {

	public:
		KinectScan(ros::NodeHandle* node);
		~KinectScan(void);


		void SubscribeTo(std::string topic);
		void onRunning(void);

	private:
		void roskinect_callback_(const sensor_msgs::LaserScan& msg);

};

	}
}


#endif
