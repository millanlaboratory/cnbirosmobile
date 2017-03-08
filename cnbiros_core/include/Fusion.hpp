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
		Fusion(ros::NodeHandle* node, std::string name = CNBIROS_FUSION_NAME);
		virtual ~Fusion(void);

		void AddSource(std::string topic);
		void SetDecayTime(float time);

	protected:
		virtual void onRunning(void);
		virtual void onStop(void);
		virtual void onStart(void);
		
		virtual void process_fusion(grid_map::GridMap& map, std::string target);
		virtual void process_decay(grid_map::GridMap& map, std::string target, float decayrate);	

	private:
		void rosgridmap_callback_(const grid_map_msgs::GridMap& msg);


	protected:
		float decayrate_;
		grid_map::GridMap 	rosgrid_;
		std::string 		fusion_layer_;
};

	}
}

#endif
