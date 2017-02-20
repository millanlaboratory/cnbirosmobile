#ifndef ROBOTINOINFRARED_HPP
#define ROBOTINOINFRARED_HPP

#include <rec/robotino/api2/DistanceSensorArray.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Robotino.hpp"
#include "RobotinoCom.hpp"
#include "Sensor.hpp"

#define DEFAULT_INFRARED_NAME "robotino_infrared"

namespace cnbiros {
	namespace robotino {



class RobotinoInfrared : public core::Sensor, public  rec::robotino::api2::DistanceSensorArray {

	public:
		RobotinoInfrared(std::string hostname, 
						 std::string name=DEFAULT_INFRARED_NAME);
		virtual ~RobotinoInfrared(void) {};
		
		void distancesChangedEvent(const float* distances, unsigned int size);

		void Read(void);
	
	protected:
		RobotinoCom* com_;

	private:
		grid_map::GridMap grid_;
		grid_map_msgs::GridMap msg_;
		

};


	}
}




#endif
