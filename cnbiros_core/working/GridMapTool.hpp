#ifndef CNBIROS_CORE_GRIDMAP_TOOL_HPP
#define CNBIROS_CORE_GRIDMAP_TOOL_HPP

#include <grid_map_ros/grid_map_ros.hpp>

namespace cnbiros {
	namespace core {

class GridMapTool {

	public:
		static void ReplaceNaN(grid_map::GridMap& map, float value);	
		static void Reset(grid_map::GridMap& map, std::string layer, float value = 0.0f);
		static void Reset(grid_map::GridMap& map, float value = 0.0f);
		static void SetGeometry(grid_map::GridMap& map, float xsize, float ysize, float res);
		static void SetFrameId(grid_map::GridMap& map, std::string frame);
		static void ToMessage(grid_map::GridMap& map, grid_map_msgs::GridMap& msg);
		static bool SumLayers(grid_map::GridMap& map, std::string target);
		static bool SumLayers(grid_map::GridMap& map, std::string target, std::vector<std::string> layers);
		static bool Exists(grid_map::GridMap& map, std::string layer);
		static void SetLayerMin(grid_map::GridMap& map, std::string layer, float minimum);
		static void SetLayerMax(grid_map::GridMap& map, std::string layer, float maximum);
		static void SetLayerMinMax(grid_map::GridMap& map, std::string layer, float minimum, float maximum);
	//	static void FillGaussian(grid_map::GridMap& map, grid_map::Position center, float radius);
};
	}
}


#endif
