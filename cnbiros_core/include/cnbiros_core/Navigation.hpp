#ifndef CNBIROS_CORE_NAVIGATION_HPP
#define CNBIROS_CORE_NAVIGATION_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_core/SensorGrid.hpp"
#include "cnbiros_services/NavigationParameter.h"

namespace cnbiros {
	namespace core {

class Navigation : public RosInterface {

	public:
		Navigation(std::string name);
		virtual ~Navigation(void);

		void AddSource(std::string topic);
		
		bool SetParameter(std::string name, float value, bool create = false);
		bool GetParameter(std::string name, float& value);
		void DumpParameters(void);

	protected:
		virtual void onStop(void);
		virtual void onStart(void);
		virtual void onRunning(void) = 0;
		virtual void onReceived(const grid_map_msgs::GridMap& msg);

	private:
		bool on_service_set_(cnbiros_services::NavigationParameter::Request& req,
							 cnbiros_services::NavigationParameter::Response& res);

	protected:
		std::map<std::string, float> 	nav_params_;
		ros::ServiceServer  			rossrv_parameter_;

		std::string 			topic_;
		SensorGrid 				grid_;
};



	}
}

#endif
