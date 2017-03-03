#ifndef CNBIROS_CORE_SENSOR_HPP
#define CNBIROS_CORE_SENSOR_HPP

#include <map>

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
		Sensor(ros::NodeHandle* node);
		virtual ~Sensor(void);

		void AdvertiseOn(std::string topic);
		void SetGrid(std::string layer, float xsize, float ysize, float res,
					 std::string frame = "base_link");	
	
	protected:
		grid_map::GridMap 		rosgrid_;
		std::string 			rosgrid_layer_;
		std::string 			rostopic_grid_;
		
};


	}
}


#endif
