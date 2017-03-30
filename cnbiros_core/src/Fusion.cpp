#ifndef CNBIROS_CORE_FUSION_CPP
#define CNBIROS_CORE_FUSION_CPP

#include "cnbiros_core/Fusion.hpp"

namespace cnbiros {
	namespace core {

Fusion::Fusion(std::string name) : RosInterface(name) {

	// Abstract fusion initialization
	this->topic_ = "/fusion_" + this->GetName();
	this->SetPublisher<grid_map_msgs::GridMap>(this->topic_);
	this->SetDecayTime(0.0f);

	// Service for fusion gridmap reset
	this->srv_reset_ = this->advertiseService("reset_" + this->GetName(), 
								&Fusion::on_service_reset_, this);

	// SensorGrid initialization
	this->grid_.SetGeometry(CNBIROS_SENSORGRID_X, CNBIROS_SENSORGRID_Y, 
							   CNBIROS_SENSORGRID_R);
	this->grid_.AddLayer(this->GetName());
	this->grid_.SetFrame(this->GetFrame());
	this->grid_.Reset();
}

Fusion::~Fusion(void) {};

void Fusion::AddSource(std::string topic, const unsigned int type) {
	switch(type) {
		case Fusion::AsLaserScan:
			this->SetSubscriber<sensor_msgs::LaserScan>(topic, 
				  boost::bind(&Fusion::onLaserScan, this, _1, topic));
			break;
		case Fusion::AsPointCloud:
			this->SetSubscriber<sensor_msgs::PointCloud>(topic, 
				  boost::bind(&Fusion::onPointCloud, this, _1, topic));
			break;
	}
}

void Fusion::onLaserScan(const sensor_msgs::LaserScan::ConstPtr& data, std::string topic) {

	sensor_msgs::LaserScan msg = *data;

	// Check if the layer already exists, otherwise add a new layer
	if(this->grid_.Exists(topic) == false) {
		this->grid_.AddLayer(topic);
		ROS_INFO("%s has added layer %s", this->GetName().c_str(), topic.c_str());
	}

	this->grid_.Reset(topic);
	this->grid_.Update(topic, msg, 0);
}

void Fusion::onPointCloud(const sensor_msgs::PointCloud::ConstPtr& data, std::string topic) {

	sensor_msgs::PointCloud msg = *data;

	// Check if the layer already exists, otherwise add a new layer
	if(this->grid_.Exists(topic) == false) {
		this->grid_.AddLayer(topic);
		ROS_INFO("%s has added layer %s", this->GetName().c_str(), topic.c_str());
	}

	this->grid_.Reset(topic);
	this->grid_.Update(topic, msg);
}


void Fusion::SetDecayTime(float time) {

  float frequency;
  this->decayrate_ = 1.0f;
  frequency = this->GetFrequency();
  
  if (time > 0.0f)
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

bool Fusion::on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res) {

	grid_map_msgs::GridMap msg;
	
	res.result = false;
	if(req.reset == true) {
		ROS_INFO("GridMap of %s requested to reset", this->GetName().c_str());
		
		if(this->IsStopped() == false) {
			this->grid_.Reset();
			msg = this->grid_.ToMessage();
			this->Publish(this->topic_, msg);
			res.result = true;
		}
	}

	return res.result;
}

void Fusion::onRunning(void) {

	grid_map_msgs::GridMap msg;

	this->process_decay(this->grid_, this->GetName(), this->decayrate_);

	// Process layer
	this->Process();

	// Publish the grid messeage
	msg = this->grid_.ToMessage();
	this->Publish(this->topic_, msg);
}

void Fusion::onStop(void) {
	grid_map_msgs::GridMap msg;
	
	this->grid_.Reset();
	msg = this->grid_.ToMessage();
	this->Publish(this->topic_, msg);
}

void Fusion::onStart(void) {
	grid_map_msgs::GridMap msg;
	
	this->grid_.Reset();
	msg = this->grid_.ToMessage();
	this->Publish(this->topic_, msg);
}

	}
}

#endif
