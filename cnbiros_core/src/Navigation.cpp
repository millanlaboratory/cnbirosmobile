#ifndef CNBIROS_CORE_NAVIGATION_CPP
#define CNBIROS_CORE_NAVIGATION_CPP

#include "Navigation.hpp"

namespace cnbiros {
	namespace core {

Navigation::Navigation(ros::NodeHandle* node, std::string name) : RosInterface(node) {

	// Abstract sensor initialization
	this->SetName(name);
	this->SetPublisher<geometry_msgs::Twist>("/cmd_vel");

	// Service for navigation parameter
	this->rossrv_param_ = this->rosnode_->advertiseService("parameter_set", 
											&Navigation::on_parameter_set_, this);
}

Navigation::~Navigation(void) {};

bool Navigation::on_parameter_set_(cnbiros_services::NavParameterSet::Request& req,
							   cnbiros_services::NavParameterSet::Response& res) {

	ROS_INFO("%s navigation requested to set parameter %s to %f", 
			 this->GetName().c_str(), req.name.c_str(), req.value);

	res.result = this->Navigation::SetParameter(req.name, req.value);
	
	if(res.result == true) {
		ROS_INFO("Parameter set for %s", this->GetName().c_str()); 
	} else {
		ROS_WARN("Cannot set parameter %s for %s: parameter does not exist", 
				  req.name.c_str(), this->GetName().c_str());
	}

	if(req.dump == true) {
		this->DumpParameters();
	}

	return res.result;
}

bool Navigation::SetParameter(std::string name, float value, bool create) {

	bool result;
	std::map<std::string, float>::iterator it;

	result = true;

	if(create == false) {
		it = this->nav_params_.find(name);
		if(it == this->nav_params_.end()) {
			result = false;
		}
	}

	if (result == true) {
		this->nav_params_[name] = value;
	}
	return result;
}

bool Navigation::GetParameter(std::string name, float& value) {

	bool result;
	std::map<std::string, float>::iterator it;

	result = false;
	it = this->nav_params_.find(name);

	if(it != this->nav_params_.end()) {
		value = it->second;
		result = true;
	}

	return result;
}

void Navigation::DumpParameters(void) {

	std::map<std::string, float>::iterator it;
	
	ROS_INFO(" + Dump parameters for %s navigation:", this->GetName().c_str());
	for(it = this->nav_params_.begin(); it != this->nav_params_.end(); ++it) {
		ROS_INFO(" |- %-20s %8f", it->first.c_str(), it->second);
	}
}

void Navigation::AddSource(std::string topic) {
	this->SetSubscriber(topic, &Navigation::rosgridmap_callback_, this);
}

void Navigation::rosgridmap_callback_(const grid_map_msgs::GridMap& msg) {
	grid_map::GridMapRosConverter::fromMessage(msg, this->rosgrid_);
}

void Navigation::onStop(void) {
	geometry_msgs::Twist msg;	
	
	this->Publish(msg);
	GridMapTool::Reset(this->rosgrid_);
}

void Navigation::onStart(void) {
	geometry_msgs::Twist msg;	
	
	this->Publish(msg);
	GridMapTool::Reset(this->rosgrid_);
}

	}
}

#endif
