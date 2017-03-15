#ifndef CNBIROS_CORE_ROSINTERFACE_CPP
#define CNBIROS_CORE_ROSINTERFACE_CPP

#include "RosInterface.hpp"

namespace cnbiros {
	namespace core {

RosInterface::RosInterface(std::string ns) : NodeHandle(ns) {
	this->name_    	  		= "rosinterface";
	this->is_stopped_ 		= false;
	this->frequency_  		= CNBIROS_NODE_FREQUENCY;
	this->rosrate_    		= new ros::Rate(this->frequency_);
	this->rosframe_ 		= "";
	//this->rosframe_child_  	= "";
	//this->rosframe_parent_ 	= "";

	// Initialize services
	this->rossrv_state_ = this->advertiseService("rosinterface_state",
								&RosInterface::on_rosinterface_service_, this); 
}

RosInterface::~RosInterface(void) {
	this->shutdown();
}

bool RosInterface::on_rosinterface_service_(cnbiros_services::RosInterfaceState::Request &req,
											cnbiros_services::RosInterfaceState::Response &res) {

	res.result = true;

	switch(req.state) {
		case RosInterfaceState::Start:
			ROS_INFO("%s interface requested to start", this->GetName().c_str());
			if(this->IsStopped() == true) {
				this->Resume();
			}
			break;
		case RosInterfaceState::Stop:
			ROS_INFO("%s interface requested to stop", this->GetName().c_str());
			if(this->IsStopped() == false) {
				this->Stop();
			}
			break;
		case RosInterfaceState::Resume:
			ROS_INFO("%s interface requested to resume", this->GetName().c_str());
			if(this->IsStopped() == true) {
				this->Resume();
			}
			break;
		default:
			ROS_INFO("Unknown request for %s interface", this->GetName().c_str());
			res.result = false;
			break;
	}

	return res.result;
}

void RosInterface::SetName(const std::string name) {
	this->name_ = name;
}

std::string RosInterface::GetName(void) {
	return this->name_;
}

void RosInterface::SetFrequency(float frequency) {
	delete this->rosrate_;
	this->frequency_ = frequency;
	this->rosrate_   = new ros::Rate(frequency);
}

float RosInterface::GetFrequency(void) {
	return this->frequency_;
}

ros::Subscriber* RosInterface::GetSubscriber(std::string topic) {
	
	auto it = this->rossubs_.find(topic);
	ros::Subscriber* ptr_sub = nullptr;

	if(it != this->rossubs_.end()) {
		ptr_sub = &it->second;
	} else {
		ROS_ERROR("[%s] - Cannot retrieve subscriber on topic %s: does not exist",
				  this->GetName().c_str(), topic.c_str());
	}

	return ptr_sub;
}

ros::Publisher* RosInterface::GetPublisher(std::string topic) {

	auto it = this->rospubs_.find(topic);
	ros::Publisher* ptr_pub = nullptr;

	if(it != this->rospubs_.end()) {
		ptr_pub = &it->second;
	} else {
		ROS_ERROR("[%s] - Cannot retrieve publisher on topic %s: does not exist",
				  this->GetName().c_str(), topic.c_str());
	}

	return ptr_pub;
}

void RosInterface::Stop(void) {
	ROS_INFO("%s interface stops", this->GetName().c_str());
	this->is_stopped_ = true;
	this->onStop();
}

void RosInterface::Resume(void) {
	ROS_INFO("%s interface starts", this->GetName().c_str());
	this->is_stopped_ = false;
	this->onStart();
}

bool RosInterface::IsStopped(void) {
	return this->is_stopped_;
}

void RosInterface::Run(void) {

	while(this->ok()) {

		if(this->IsStopped() == false) {
			
			this->onRunning();
		}

		rosrate_->sleep();
		ros::spinOnce();
	}

	ros::spin();
}

void RosInterface::onRunning(void) {}

//void RosInterface::SetParentFrame(std::string frameid) {
//	this->rosframe_parent_ = frameid;
//}
//
//std::string RosInterface::GetParentFrame(void) {
//	return this->rosframe_parent_;
//}
//
//void RosInterface::SetChildFrame(std::string frameid) {
//	this->rosframe_child_ = frameid;
//}
//
//std::string RosInterface::GetChildFrame(void) {
//	return this->rosframe_child_;
//}

void RosInterface::SetFrame(const std::string frame) {
	this->rosframe_ = frame;
}

std::string RosInterface::GetFrame(void) {
	return this->rosframe_;
}

	}
}


#endif
