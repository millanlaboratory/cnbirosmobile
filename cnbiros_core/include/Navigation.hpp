#ifndef CNBIROS_CORE_NAVIGATION_HPP
#define CNBIROS_CORE_NAVIGATION_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"
#include "RosInterface.hpp"
#include "GridMapTool.hpp"
#include "cnbiros_services/NavParameterSet.h"

namespace cnbiros {
	namespace core {

class Navigation : public RosInterface {

	public:
		Navigation(ros::NodeHandle* node, std::string name);
		virtual ~Navigation(void);

		void AddSource(std::string topic);
		
		bool SetParameter(std::string name, float value, bool create = false);
		bool GetParameter(std::string name, float& value);
		void DumpParameters(void);

	protected:
		virtual void onStop(void);
		virtual void onStart(void);
		virtual void onRunning(void) = 0;
		virtual void rosgridmap_callback_(const grid_map_msgs::GridMap& msg);

	private:
		bool on_parameter_set_(cnbiros_services::NavParameterSet::Request& req,
							   cnbiros_services::NavParameterSet::Response& res);

	protected:
		std::map<std::string, float> 	nav_params_;
		ros::ServiceServer  			rossrv_param_;

		std::string 			grid_layer_;
		grid_map::GridMap 		rosgrid_;
};



	}
}

#endif
