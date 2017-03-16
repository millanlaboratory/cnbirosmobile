#ifndef CNBIROS_CORE_GRIDMAP_TOOL_CPP
#define CNBIROS_CORE_GRIDMAP_TOOL_CPP

#include "GridMapTool.hpp"

namespace cnbiros {
	namespace core {


void GridMapTool::ReplaceNaN(grid_map::GridMap& map, float value) {
	std::vector<std::string> layers;
	std::vector<std::string>::iterator it;

	layers = map.getLayers();

	for (it = layers.begin(); it != layers.end(); ++it) {
		grid_map::Matrix& data = map[*it];
		data = (data.array().isNaN()).select(0.0f, data);
	}
}

void GridMapTool::Reset(grid_map::GridMap& map, std::string layer, float value) {
	if(GridMapTool::Exists(map, layer))
		map[layer].setConstant(value);
}

void GridMapTool::Reset(grid_map::GridMap& map, float value) {
	
	std::vector<std::string> layers = map.getLayers();
	
	for(auto it = layers.begin(); it != layers.end(); ++it) {
		map[*it].setConstant(value);
	}
}

void GridMapTool::SetGeometry(grid_map::GridMap& map, float xsize, float ysize, float res) {
	map.setGeometry(grid_map::Length(xsize, ysize), res);
}

void GridMapTool::SetFrameId(grid_map::GridMap& map, std::string frame) {
	map.setFrameId(frame);
}

void GridMapTool::ToMessage(grid_map::GridMap& map, grid_map_msgs::GridMap& msg) {
	grid_map::GridMapRosConverter::toMessage(map, msg);
}

bool GridMapTool::SumLayers(grid_map::GridMap& map, std::string target) {

	bool result;
	std::vector<std::string> layers;
	layers = map.getLayers();

	result = GridMapTool::SumLayers(map, target, layers);
	
	return result;
}

bool GridMapTool::SumLayers(grid_map::GridMap& map, std::string target, std::vector<std::string> layers) {

	bool result = true;
	std::vector<std::string>::iterator it;

	if(GridMapTool::Exists(map, target) == false) {
		ROS_WARN("Target layer (%s) does not exist. Cannot fuse required layers", target.c_str());
		return false;
	}

	for(it = layers.begin(); it != layers.end(); ++it) {
		if(map.exists(*it) == false) {
			ROS_WARN("Layer %s does not exist in the grid map. Skipped.", (*it).c_str());
			result = false;
		} else {
			if((*it).compare(target) != 0) {
				map[target] += map[*it];
			}
		}
	}

	return result;
}

bool GridMapTool::Exists(grid_map::GridMap& map, std::string layer) {

	bool result = true;
	if(map.exists(layer) == false) {
		ROS_WARN("Layer (%s) does not exist", layer.c_str());
		result = false;
	}
	return result;
}

void GridMapTool::SetLayerMax(grid_map::GridMap& map, std::string layer, float maximum) {
	if(GridMapTool::Exists(map, layer) == true) {
		map[layer] = (map[layer].array() > maximum).select(maximum, map[layer]);
	}
}

void GridMapTool::SetLayerMin(grid_map::GridMap& map, std::string layer, float minimum) {
	if(GridMapTool::Exists(map, layer) == true) {
		map[layer] = (map[layer].array() < minimum).select(minimum, map[layer]);
	}
}

void GridMapTool::SetLayerMinMax(grid_map::GridMap& map, std::string layer, float minimum, float maximum) {
	GridMapTool::SetLayerMin(map, layer, minimum);
	GridMapTool::SetLayerMax(map, layer, maximum);
}

	}
}


#endif
