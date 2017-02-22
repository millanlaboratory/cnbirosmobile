#ifndef SENSORSFUSION_HPP
#define SENSORSFUSION_HPP

#include <string>
#include <map>
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"

namespace cnbiros {
	namespace core {

class SensorsFusion {
	
	public:
		SensorsFusion(float frequency=CNBIROS_SENSOR_NODE_FREQUENCY);
		virtual ~SensorsFusion(void);

		void Register(ros::NodeHandle* node);
		bool IsRegistered(void);
		void Advertise(std::string topic = "/sensors_fusion");
		void Subscribe(std::string topic);

		void SetGrid(std::string layer, std::string frameid);
		void SetGrid(float x, float y, float r);

		virtual void Run(void);


	protected:
		virtual void fuse_layers(std::string target);
		virtual void add_layer_callback(const grid_map_msgs::GridMap& msg);


	protected:
	
		std::string 			layer_;
		std::string 			frameid_;
		float 					frequency_;
		grid_map::GridMap 		grid_;
		grid_map_msgs::GridMap 	grid_msg_;

		// ros related members
		ros::Rate* 			rosrate_;
		ros::NodeHandle* 	rosnode_;
		ros::Publisher  	rospub_;
		std::string 		rospub_topic_;
		std::map<std::string, ros::Subscriber> rossublist_;


};

	}
}

#endif
