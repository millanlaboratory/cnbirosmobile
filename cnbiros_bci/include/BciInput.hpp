#ifndef CNBIROS_BCI_BCIINPUT_HPP
#define CNBIROS_BCI_BCIINPUT_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "cnbiros_messages/TobiId.h"
#include "Flags.hpp"
#include "RosInterface.hpp"
#include "GridMapTool.hpp"

namespace cnbiros {
	namespace bci {

class BciInput : public core::RosInterface {

	public: 
		BciInput(void);
		~BciInput(void);
	
		void AdvertiseOn(std::string topic);

		void SetGrid(std::string layer, float xsize, float ysize, float res,
					 std::string frame = "base_link");	

		void onRunning(void);

		float GetOrientation(nav_msgs::Odometry& msg);

		void ResetDiscrete(float time);
	protected:
		void on_tobiid_received_(const cnbiros_messages::TobiId& msg);
		void on_tobiic_received_(const cnbiros_messages::TobiId& msg){};	// <- to be changed type of message
		void on_odometry_received_(const nav_msgs::Odometry& msg);

	protected:
		grid_map::GridMap 		rosgrid_;
		std::string 			rostopic_grid_;
		ros::Time				received_time_;
		float 					target_angle_;
		nav_msgs::Odometry 		rosodom_msg_;

};

	}
}

#endif
