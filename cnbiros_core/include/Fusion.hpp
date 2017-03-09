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
#include "cnbiros_services/GridMapReset.h"

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
		bool on_gridmap_reset_(cnbiros_services::GridMapReset::Request& req,
							  cnbiros_services::GridMapReset::Response& res);

	protected:
		float 				decayrate_;
		std::string 		fusion_layer_;
		grid_map::GridMap 	rosgrid_;

	private:
		ros::ServiceServer	rossrv_reset_;
};

	}
}

#endif
