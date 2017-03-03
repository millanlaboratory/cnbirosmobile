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
	if(map.exists(layer))
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

grid_map_msgs::GridMap GridMapTool::ToMessage(grid_map::GridMap& map) {
	grid_map_msgs::GridMap msg;
	grid_map::GridMapRosConverter::toMessage(map, msg);
	return msg;
}


	}
}


#endif
