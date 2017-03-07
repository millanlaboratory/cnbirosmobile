#ifndef CNBIROS_CORE_NAVIGATION_HPP
#define CNBIROS_CORE_NAVIGATION_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"
#include "RosInterface.hpp"

namespace cnbiros {
	namespace core {

class Navigation : public RosInterface {

	public:
		Navigation(ros::NodeHandle* node);
		virtual ~Navigation(void);

		void SubscribeTo(std::string topic);
		void AdvertiseOn(std::string topic);

	protected:
		virtual void rosgridmap_callback_(const grid_map_msgs::GridMap& msg);

	protected:
		bool has_message_;	
		grid_map_msgs::GridMap 	rosgridmap_msg_;
		geometry_msgs::Twist 	rostwist_msg_;
};



	}
}

#endif
