#ifndef CNBIROS_CORE_NAVIGATION_HPP
#define CNBIROS_CORE_NAVIGATION_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "cnbiros_core/Flags.hpp"
#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_core/SensorGrid.hpp"
#include "cnbiros_services/NavParameterSet.h"

namespace cnbiros {
	namespace core {

class Navigation : public RosInterface {

	public:
		Navigation(std::string name);
		virtual ~Navigation(void);

		void AddSource(std::string topic);
		
		bool SetNavigationParameter(std::string name, float value, bool create = false);
		bool GetNavigationParameter(std::string name, float& value);
		void DumpParameters(void);

	protected:
		virtual void onStop(void);
		virtual void onStart(void);
		virtual void onRunning(void) = 0;
		virtual void rosnavigation_callback_(const grid_map_msgs::GridMap& msg);

	private:
		bool on_service_setparameter_(cnbiros_services::NavParameterSet::Request& req,
							   cnbiros_services::NavParameterSet::Response& res);

	protected:
		std::map<std::string, float> 	nav_params_;
		ros::ServiceServer  			rossrv_parameter_;

		std::string 			rostopic_pub_;
		SensorGrid 				rosgrid_;
};



	}
}

#endif
