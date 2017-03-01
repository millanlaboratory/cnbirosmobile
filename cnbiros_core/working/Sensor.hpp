#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"

namespace cnbiros {
	namespace core {

class Sensor {

	public:
		Sensor(ros::NodeHandle* node);
		virtual ~Sensor(void);

		void SetFrequency(float frequency);	
		float GetFrequency(void);


		void SetFrames(std::string frameid, std::string child_frameid);

		void SetGrid(std::string layer, float x, float y, float r);
		bool IsGridSet(void);


		virtual void Run(void) = 0;

	protected:
		float 		frequency_;

		// Transform members
		std::string 	frameid_;
		std::string 	child_frameid_;
		tf::TransformBroadcaster 	broadcaster_;
		tf::TransformListener 		listener_;
		
		// ros related members
		ros::NodeHandle* rosnode_;
		std::string 	 rostopic_sub_;
		std::string 	 rostopic_pub_;
		ros::Publisher 	 rospub_;
		ros::Subscriber  rossub_;
		ros::Rate* 		 rosrate_;

		// grid related members
		std::string 			grid_layer_;
		grid_map::GridMap 		grid_;
		grid_map_msgs::GridMap 	grid_msg_;
		bool 					grid_is_set_;

};
	}
}

#endif
