#ifndef CNBIROS_CORE_KINECTSCAN_HPP
#define CNBIROS_CORE_KINECTSCAN_HPP

#include <sensor_msgs/LaserScan.h>

#include "Flags.hpp"
#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

class KinectScan : public Sensor {

	public:
		KinectScan(ros::NodeHandle* node, std::string name = CNBIROS_KINECTSCAN_NAME);
		~KinectScan(void);

		void onRunning(void);

	private:
		void roskinect_callback_(const sensor_msgs::LaserScan& msg);

};

	}
}


#endif
