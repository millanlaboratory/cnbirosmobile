#ifndef CNBIROS_CORE_NAVIGATION_CPP
#define CNBIROS_CORE_NAVIGATION_CPP

#include "cnbiros_core/Navigation.hpp"

namespace cnbiros {
	namespace core {

Navigation::Navigation(std::string name) : RosInterface(name) {

	std::vector<std::string> sources;
	const std::vector<std::string> defsources = {"/fusion"};
	
	// Abstract sensor initialization
	this->rostopic_pub_ = "/cmd_vel";
	this->SetPublisher<geometry_msgs::Twist>(this->rostopic_pub_);

	// Add sources
	this->GetParameter("sources", sources, defsources);
	for(auto it = sources.begin(); it != sources.end(); ++it) {
		ROS_INFO("Added source for '%s': %s", this->GetName().c_str(), (*it).c_str());
		this->AddSource(*it);
	}

	// Service for navigation parameter
	this->rossrv_parameter_ = this->advertiseService("set_parameter", 
							  &Navigation::on_service_setparameter_, this);
}

Navigation::~Navigation(void) {};

bool Navigation::on_service_setparameter_(cnbiros_services::NavParameterSet::Request& req,
							   cnbiros_services::NavParameterSet::Response& res) {

	ROS_INFO("%s navigation requested to set parameter %s to %f", 
			 this->GetName().c_str(), req.name.c_str(), req.value);

	res.result = this->SetNavigationParameter(req.name, req.value);
	
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

bool Navigation::SetNavigationParameter(std::string name, float value, bool create) {

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

bool Navigation::GetNavigationParameter(std::string name, float& value) {

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
	this->SetSubscriber(topic, &Navigation::rosnavigation_callback_, this);
}

void Navigation::rosnavigation_callback_(const grid_map_msgs::GridMap& msg) {
	
	grid_map::GridMap grid;
	
	if(grid_map::GridMapRosConverter::fromMessage(msg, grid) == false)
		ROS_WARN("%s cannot convert message to sensor grid", this->GetName().c_str());

	this->rosgrid_.addDataFrom(grid, true, true, true);
}

void Navigation::onStop(void) {
	geometry_msgs::Twist msg;	
	
	this->Publish(this->rostopic_pub_, msg);
	this->rosgrid_.Reset();
}

void Navigation::onStart(void) {
	geometry_msgs::Twist msg;	
	
	this->Publish(this->rostopic_pub_, msg);
	this->rosgrid_.Reset();
}

	}
}

#endif
