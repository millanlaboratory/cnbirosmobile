#ifndef CNBIROS_CORE_NAVIGATION_HPP
#define CNBIROS_CORE_NAVIGATION_HPP

#include <string>
#include <map>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"

namespace cnbiros {
	namespace core {

class Navigation {

	public:
		Navigation(ros::NodeHandle* node);
		virtual ~Navigation(void);

		void GetName(std::string& name);
		void GetFrequency(float& frequency);
		void SetName(std::string name);
		void SetFrequency(float frequency);

		void SetPublisher(std::string topic);
		void SetSubscriber(std::string topic, unsigned int type);

		void PublishTwist(void);
		virtual void Run(void) = 0;
	
	protected:
		virtual void rosodometry_callback(const nav_msgs::Odometry& msg);
		virtual void rosgridmap_callback(const grid_map_msgs::GridMap& msg);

	public:
		enum MsgType { AsOdometry, AsGridMap };

	protected:
		std::string name_;
		float 		frequency_;
		
		grid_map_msgs::GridMap 	rosgridmap_msg_;
		nav_msgs::Odometry		rosodometry_msg_; 
		geometry_msgs::Twist 	rostwist_msg_;

		// ros related members
		ros::Rate* 			rosrate_;
		ros::NodeHandle* 	rosnode_;
		ros::Publisher  	rospub_;
		ros::Subscriber		rossub_odometry_;
		ros::Subscriber		rossub_gridmap_;
};


	}
}

#endif
