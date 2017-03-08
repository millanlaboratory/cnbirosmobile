#ifndef CNBIROS_CORE_FUSION_CPP
#define CNBIROS_CORE_FUSION_CPP

#include "Fusion.hpp"

namespace cnbiros {
	namespace core {

Fusion::Fusion(ros::NodeHandle* node, std::string name) : RosInterface(node) {

	// Abstract fusion initialization
	this->SetName(name);
	this->SetPublisher<grid_map_msgs::GridMap>("/" + this->GetName());
	this->SetDecayTime(0.0f);

	// GridMap initialization
	this->rosgrid_.add(this->GetName());
	this->fusion_layer_ = this->GetName();	
	GridMapTool::SetGeometry(this->rosgrid_, CNBIROS_GRIDMAP_XSIZE, 
							 CNBIROS_GRIDMAP_YSIZE, CNBIROS_GRIDMAP_RESOLUTION);
	GridMapTool::SetFrameId(this->rosgrid_, CNBIROS_GRIDMAP_FRAME);
	GridMapTool::Reset(this->rosgrid_, this->GetName());
}

Fusion::~Fusion(void) {};

void Fusion::AddSource(std::string topic) {
	this->SetSubscriber(topic, &Fusion::rosgridmap_callback_, this);
}

void Fusion::rosgridmap_callback_(const grid_map_msgs::GridMap& msg) {
	std::vector<std::string> layers;
	grid_map::GridMap grid;
	float src_resolution, dst_resolution;

	// Convert grid message to the grid object 
	if(grid_map::GridMapRosConverter::fromMessage(msg, grid) == false)
		ROS_WARN("Cannot convert message to grid");

	// Get the current grid resolution
	src_resolution = grid.getResolution();
	dst_resolution = this->rosgrid_.getResolution();

	// If different change resolution of the incoming map
	if(src_resolution != dst_resolution) {		
		ROS_WARN_THROTTLE(10, "Incoming grid map has different resolution, "
						   	   "grid map discarded. Set the same resolution "
						   	   "for all incoming grid maps");
	} else {
	
		// Add all layers from incoming grid to the local grid 
		this->rosgrid_.addDataFrom(grid, true, true, true);
	}

	// Replace NaN values (coming from possible resize of the incoming grid) 
	GridMapTool::ReplaceNaN(this->rosgrid_, 0.0f);
}

void Fusion::SetDecayTime(float time) {

	float frequency;
	this->decayrate_ = 1.0f;
	frequency = this->GetFrequency();
	
	if (time > 0)
		this->decayrate_ = 1.0f/(frequency*time);
}

void Fusion::process_decay(grid_map::GridMap& map, std::string target, float decayrate) {
	
	unsigned int nrows, ncols;
	Eigen::MatrixXf mdecay;
	
	nrows = map[target].rows();
	ncols = map[target].cols();

	mdecay = Eigen::MatrixXf::Constant(nrows, ncols, decayrate);

	map[target] = (map[target].array() > 0.0f).select(
				   map[target]-mdecay , map[target]);
	map[target] = (map[target].array() < 0.0f).select(
				   map[target]+mdecay , map[target]);
}

void Fusion::process_fusion(grid_map::GridMap& map, std::string target) {
	GridMapTool::SumLayers(map, target);
	GridMapTool::SetLayerMinMax(map, target, -1.0f, 1.0f);
}


void Fusion::onRunning(void) {
	
	grid_map_msgs::GridMap msg;
	
	// Process decay 
	this->process_decay(this->rosgrid_, this->fusion_layer_, this->decayrate_);

	// Fuse layers and limit the values between -1 and 1
	this->process_fusion(this->rosgrid_, this->fusion_layer_);

	// Publish the grid messeage
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(msg);
}

void Fusion::onStop(void) {
	grid_map_msgs::GridMap msg;
	
	GridMapTool::Reset(this->rosgrid_);
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(msg);
}

void Fusion::onStart(void) {
	grid_map_msgs::GridMap msg;
	
	GridMapTool::Reset(this->rosgrid_);
	GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(msg);
}

	}
}

#endif
