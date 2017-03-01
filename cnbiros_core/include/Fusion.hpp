#ifndef CNBIROS_CORE_FUSION_HPP
#define CNBIROS_CORE_FUSION_HPP

#include <string>
#include <map>
#include <ros/ros.h>
#include <ros/console.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

class Fusion : public Sensor {
	
	public:
		Fusion(ros::NodeHandle* node);
		virtual ~Fusion(void);

		void SetSubscriber(std::string topic);
		void DeleteSubscriber(std::string topic);

		void SetDecayTime(float time);

		virtual void Run(void);


	protected:
		virtual void rossensor_callback(const grid_map_msgs::GridMap& msg);
		virtual void FuseLayersTo(std::string target);
		virtual void ProcessPersistency(std::string target);


		//virtual void fuse_layers(std::string target);
		//virtual void add_layer_callback(const grid_map_msgs::GridMap& msg);


	protected:
	
		std::map<std::string, ros::Subscriber> rossub_list_;
		
		float decayrate_;

};

	}
}

#endif
