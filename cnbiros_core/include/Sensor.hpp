#ifndef CNBIROS_CORE_SENSOR_HPP
#define CNBIROS_CORE_SENSOR_HPP

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"
#include "RosInterface.hpp"
#include "GridMapTool.hpp"

namespace cnbiros {
	namespace core {

class Sensor : public RosInterface {

	public:
		Sensor(ros::NodeHandle* node, std::string name);
		virtual ~Sensor(void);

	protected:
		virtual void onStop(void);
		virtual void onStart(void);
		virtual void onRunning(void) = 0;

	protected:
		grid_map::GridMap 		rosgrid_;
		std::string 			sensor_layer_;
		std::string 			rostopic_grid_;
		
};


	}
}


#endif
