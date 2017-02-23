#ifndef NAVIGATIONCTRL_HPP
#define NAVIGATIONCTRL_HPP

#include <string>
#include <map>
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"
#include "cnbiros_messages/RobotVelocity.h"
#include "cnbiros_messages/RobotOdometry.h"

namespace cnbiros {
	namespace core {

class NavigationCtrl {

	public:
		NavigationCtrl(float frequency);
		virtual ~NavigationCtrl(void);

		void Register(ros::NodeHandle* node);
		bool IsRegistered(void);

		void SubscribeSensors(std::string topic = CNBIROS_TOPIC_FUSION);
		void SubscribeOdometry(std::string topic = CNBIROS_TOPIC_ODOMETRY);
		void AdvertiseVelocity(std::string topic = CNBIROS_TOPIC_VELOCITY);

		virtual void Run(void) = 0;

	protected:
		virtual void message_odometry_callback(const cnbiros_messages::RobotOdometry& msg);
		virtual void message_sensors_callback(const grid_map_msgs::GridMap& msg);

	protected:

		float frequency_;
		
		float odom_x_;
		float odom_y_;
		float odom_z_;
		float odom_o_;
		grid_map::GridMap 		grid_;

		float vx_;
		float vy_;
		float vz_;
		float vo_;
		
		// ros related members
		ros::Rate* 			rosrate_;
		ros::NodeHandle* 	rosnode_;
		ros::Publisher  	rospub_velocity_;
		ros::Subscriber  	rossub_odometry_;
		ros::Subscriber  	rossub_sensors_;
		std::string 		rostopic_velocity_;
		std::string 		rostopic_odometry_;
		std::string 		rostopic_sensors_;

};


	}
}

#endif
