#ifndef CNBIROS_CORE_FUSION_CPP
#define CNBIROS_CORE_FUSION_CPP

#include "cnbiros_core/Fusion.hpp"

namespace cnbiros {
	namespace core {

Fusion::Fusion(std::string name) : RosInterface(name) {

	float decay;
	std::vector<std::string> sources;
	const std::vector<std::string> defsources = {"/infrared", "/kinectscan"};

	// Abstract fusion initialization
	this->rostopic_pub_ = "/" + this->GetName();
	this->SetPublisher<grid_map_msgs::GridMap>(this->rostopic_pub_);

	// Get parameter from server
	this->GetParameter("decay", decay, 0.0f);
	this->SetDecayTime(decay);
	
	// Add sources
	this->GetParameter("sources", sources, defsources);
	for(auto it = sources.begin(); it != sources.end(); ++it) {
		ROS_INFO("Added source for '%s': %s", this->GetName().c_str(), (*it).c_str());
		this->AddSource(*it);
	}

	// Service for fusion gridmap reset
	this->rossrv_reset_ = this->advertiseService("fusion_reset", 
											&Fusion::on_service_reset_, this);

	// SensorGrid initialization
	this->rosgrid_.SetGeometry(CNBIROS_SENSORGRID_X, CNBIROS_SENSORGRID_Y, 
							   CNBIROS_SENSORGRID_R);
	this->rosgrid_.AddLayer(this->GetName());
	this->rosgrid_.SetFrame(this->GetFrame());
	this->rosgrid_.Reset();
}

Fusion::~Fusion(void) {};

bool Fusion::on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res) {

	grid_map_msgs::GridMap msg;
	
	res.result = false;
	if(req.reset == true) {
		ROS_INFO("GridMap of %s requested to reset", this->GetName().c_str());
		
		if(this->IsStopped() == false) {
			this->rosgrid_.Reset();
			msg = this->rosgrid_.ToMessage();
			this->Publish(this->rostopic_pub_, msg);
			res.result = true;
		}
	}

	return res.result;
}

void Fusion::AddSource(std::string topic) {
	this->SetSubscriber(topic, &Fusion::rosfusion_callback_, this);
}

void Fusion::rosfusion_callback_(const grid_map_msgs::GridMap& msg) {

	std::vector<std::string> layers;
	SensorGrid grid;
	float src_resolution, dst_resolution;

	// Convert grid message to the grid object 
	if(grid_map::GridMapRosConverter::fromMessage(msg, grid) == false)
		ROS_WARN("%s cannot convert message to sensor grid", this->GetName().c_str());

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
	this->rosgrid_.ReplaceNaN(0.0f);
}

void Fusion::SetDecayTime(float time) {

	float frequency;
	this->decayrate_ = 1.0f;
	frequency = this->GetFrequency();
	
	if (time > 0)
		this->decayrate_ = 1.0f/(frequency*time);
}

void Fusion::process_decay(SensorGrid& grid, std::string target, float decayrate) {
	
	unsigned int nrows, ncols;
	Eigen::MatrixXf mdecay;
	
	nrows = grid[target].rows();
	ncols = grid[target].cols();

	mdecay = Eigen::MatrixXf::Constant(nrows, ncols, decayrate);

	grid[target] = (grid[target].array() > 0.0f).select(
				   grid[target]-mdecay , grid[target]);
	grid[target] = (grid[target].array() < 0.0f).select(
				   grid[target]+mdecay , grid[target]);
}

void Fusion::process_fusion(SensorGrid& grid, std::string target) {
	grid.Sum(target);
	grid.SetMinMax(target, -1.0f, 1.0f);
}


void Fusion::onRunning(void) {
	
	grid_map_msgs::GridMap msg;
	
	// Process decay 
	this->process_decay(this->rosgrid_, this->GetName(), this->decayrate_);

	// Fuse layers and limit the values between -1 and 1
	this->process_fusion(this->rosgrid_, this->GetName());

	// Publish the grid messeage
	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);
}

void Fusion::onStop(void) {
	grid_map_msgs::GridMap msg;
	
	this->rosgrid_.Reset();
	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);
}

void Fusion::onStart(void) {
	grid_map_msgs::GridMap msg;
	
	this->rosgrid_.Reset();
	msg = this->rosgrid_.ToMessage();
	this->Publish(this->rostopic_pub_, msg);
}

	}
}

#endif
