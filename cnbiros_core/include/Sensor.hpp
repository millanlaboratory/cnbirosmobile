#ifndef CNBIROS_CORE_SENSOR_HPP
#define CNBIROS_CORE_SENSOR_HPP

#include <map>

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp"

namespace cnbiros {
	namespace core {

class Sensor {
	public:
		Sensor(ros::NodeHandle* node);
		virtual ~Sensor(void);

		void GetName(std::string& name);
		void GetFrequency(float& frequency);
		void GetPublishTopics(std::vector<std::string>& topics);
		void GetPublisher(ros::Publisher& publisher, std::string topic);
		void GetGrid(grid_map::GridMap& grid);
		
		void SetName(std::string name);
		void SetFrequency(float frequency);
		void SetPublisher(std::string topic);
		void SetGrid(std::string layer, float xsize, 
					 float ysize, float res, std::string frame = "base_link");

		void DeletePublisher(std::string topic);

		void ReplaceNaN(grid_map::GridMap& map, float value);

		void PublishGrid(void);

		virtual void Run(void) = 0;

	protected:
		void ResetLayer(std::string layer, float value = 0.0f);

	protected:
		std::string 	name_;
		float 			frequency_;
		
		ros::Rate* 			rosrate_;
		ros::NodeHandle*	rosnode_;
		std::map<std::string, ros::Publisher> rospub_list_;

		grid_map::GridMap 		rosgrid_;
		grid_map_msgs::GridMap 	rosgrid_msg_;
		std::string 			rosgrid_layer_;
		std::string 			rosgrid_frame_;
		
};


	}
}


#endif
