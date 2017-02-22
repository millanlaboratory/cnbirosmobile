#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string>
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"

namespace cnbiros {
	namespace core {

class Sensor {

	public:
		Sensor(float frequency=CNBIROS_SENSOR_NODE_FREQUENCY);
		virtual ~Sensor(void);
		
		void Register(ros::NodeHandle* node);
		bool IsRegistered(void);
		void Advertise(std::string topic);

		void SetGrid(std::string layer, std::string frame);
		void SetGrid(float x, float y, float r);
		
		virtual void Process(void) = 0;

	protected:
		float 	frequency_;
		
		// ros related members
		ros::NodeHandle* rosnode_;
		std::string 	 rostopic_;
		std::string 	 rosname_;
		ros::Publisher 	 rospub_;
		ros::Rate* 		 rosrate_;

		// grid related members
		std::string layer_;
		std::string frameid_;
		grid_map::GridMap 		grid_;
		grid_map_msgs::GridMap 	grid_msg_;

};
	}
}

#endif
