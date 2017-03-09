#ifndef CNBIROS_CORE_SENSOR_HPP
#define CNBIROS_CORE_SENSOR_HPP

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"
#include "RosInterface.hpp"
#include "GridMapTool.hpp"
#include "cnbiros_services/GridMapReset.h"

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

	private:
		bool on_gridmap_reset_(cnbiros_services::GridMapReset::Request& req,
							  cnbiros_services::GridMapReset::Response& res);

	protected:
		grid_map::GridMap 		rosgrid_;
		std::string 			sensor_layer_;
		std::string 			rostopic_grid_;

	private:
		ros::ServiceServer 		rossrv_reset_;
		
};


	}
}


#endif
