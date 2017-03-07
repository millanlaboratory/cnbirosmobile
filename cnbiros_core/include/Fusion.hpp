#ifndef CNBIROS_CORE_FUSION_HPP
#define CNBIROS_CORE_FUSION_HPP

#include <string>
#include <map>
#include <ros/ros.h>
#include <ros/console.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "RosInterface.hpp"
#include "GridMapTool.hpp"

namespace cnbiros {
	namespace core {

class Fusion : public RosInterface {
	
	public:
		Fusion(ros::NodeHandle* node);
		virtual ~Fusion(void);

		void SubscribeTo(std::string topic);
		void AdvertiseOn(std::string topic);

		void SetDecayTime(float time);

		void SetGrid(std::string layer, float xsize, 
					 float ysize, float res, std::string frame = "base_link");

		void onRunning(void);

	protected:
		virtual void rosgridmap_callback_(const grid_map_msgs::GridMap& msg);
		virtual void FuseLayersTo(std::string target);
		virtual void ProcessPersistency(std::string target);

	protected:
		float decayrate_;
		grid_map::GridMap 	rosgrid_;
		std::string 		rosgrid_layer_;
};

	}
}

#endif
