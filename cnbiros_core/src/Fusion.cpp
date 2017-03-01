#ifndef CNBIROS_CORE_FUSION_CPP
#define CNBIROS_CORE_FUSION_CPP

#include "Fusion.hpp"

namespace cnbiros {
	namespace core {

Fusion::Fusion(ros::NodeHandle* node) : Sensor(node) {
	this->SetName("fusion");
	this->SetDecayTime(0.0f);

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
								   ros::console::levels::Debug);
};

Fusion::~Fusion(void) {};

void Fusion::SetSubscriber(std::string topic) {
	this->rossub_list_[topic] = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, 
												&Fusion::rossensor_callback, this);
	
	ROS_INFO("%s subscribed on topic %s", this->name_.c_str(), topic.c_str());
}

void Fusion::DeleteSubscriber(std::string topic) {
	std::map<std::string, ros::Subscriber>::iterator it;

	it = this->rossub_list_.find(topic);

	if(it != this->rossub_list_.end()) {
		this->rossub_list_.erase(it);
	}
}


void Fusion::rossensor_callback(const grid_map_msgs::GridMap& msg) {
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
		ROS_DEBUG_THROTTLE(10, "Incoming grid map has different resolution, "
						   	   "grid map discarded. Set the same resolution "
						   	   "for all incoming grid maps");
	} else {
	
		// Add all layers from incoming grid to the local grid 
		this->rosgrid_.addDataFrom(grid, true, true, true);
	}

	// Replace NaN values (coming from possible resize of the incoming grid) 
	this->ReplaceNaN(this->rosgrid_, 0.0f);
}

void Fusion::FuseLayersTo(std::string target) {

	std::vector<std::string> layers;
	std::vector<std::string>::iterator it;

	//this->rosgrid_[target].setConstant(0.0f);
	layers = this->rosgrid_.getLayers();

	for(it = layers.begin(); it != layers.end(); ++it) {
		if((*it).compare(target) != 0) {
			this->rosgrid_[target] += this->rosgrid_[*it];
		}
	}

	// Normalization with respect to the maximum [0 1]
	// TO THINK ABOUT
	//this->rosgrid_[target] /= this->rosgrid_[target].maxCoeff();
	
	this->rosgrid_[target] = (this->rosgrid_[target].array() > 1.0f).select(1, this->rosgrid_[target]);
}

void Fusion::SetDecayTime(float time) {

	float frequency;
	this->decayrate_ = 1.0f;
	this->GetFrequency(frequency);
	
	if (time > 0)
		this->decayrate_ = 1.0f/(frequency*time);
}

void Fusion::ProcessPersistency(std::string target) {
	
	unsigned int nrows, ncols;
	nrows = this->rosgrid_[target].rows();
	ncols = this->rosgrid_[target].cols();

	this->rosgrid_[target] -= Eigen::MatrixXf::Constant(nrows, ncols, this->decayrate_);
	this->rosgrid_[target] = (this->rosgrid_[target].array() < 0.0f).select(0, this->rosgrid_[target]);
}

void Fusion::Run(void) {
	
	while(this->rosnode_->ok()) {

		// Process layer persistency
		this->ProcessPersistency(this->rosgrid_layer_);

		// Fuse layers
		this->FuseLayersTo(this->rosgrid_layer_);

		// Publish the msg
		this->PublishGrid();
		
		ros::spinOnce();
		this->rosrate_->sleep();
	}
}

	}
}

#endif
