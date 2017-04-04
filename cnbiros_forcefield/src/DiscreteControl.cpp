#ifndef CNBIROS_FORCEFIELD_DISCRETE_CPP
#define CNBIROS_FORCEFIELD_DISCRETE_CPP

#include "cnbiros_forcefield/DiscreteControl.hpp"

namespace cnbiros {
	namespace forcefield {

DiscreteControl::DiscreteControl(std::string name) : RosInterface(name) {
	this->topic_ = "/" + this->GetName();
	this->flt_is_set_ = false;
	this->flt_pipe_   = "";
	this->flt_family_ = -1;


	this->SetPublisher<sensor_msgs::PointCloud>(this->topic_);

	// Service for sensor reset
	this->rossrv_reset_ = this->advertiseService("reset_control", 
											&DiscreteControl::on_service_reset_, this);
	
	this->Reset();	
}

DiscreteControl::~DiscreteControl(void) {}

void DiscreteControl::AddSource(std::string topic) {
	this->SetSubscriber(topic, &DiscreteControl::onReceived, this);
}

void DiscreteControl::SetRadius(float radius) {
	this->radius_ = radius;
}

void DiscreteControl::SetFilter(std::string pipe, int family) {
	this->flt_pipe_   = pipe;
	this->flt_family_ = family;
	this->flt_is_set_ = true;
}

void DiscreteControl::SetCommand(std::string event, float angle) {
	this->cmd_angles_[event] = angle;
}

bool DiscreteControl::CheckMessage(const cnbiros_bci::TiDMessage& msg) {

	cnbiros::bci::TobiIdTools tools(msg);
	bool is_valid = true;

	if(this->flt_is_set_ == true) {
		is_valid = tools.IsFromPipe(this->flt_pipe_);
	}

	return is_valid;
}

void DiscreteControl::onReceived(const cnbiros_bci::TiDMessage& msg) {

	std::map<std::string, float>::iterator it;
	geometry_msgs::Point32 		point;
	sensor_msgs::ChannelFloat32 channel;

	if(this->CheckMessage(msg)) {
		this->Reset();
		it = this->cmd_angles_.find(std::to_string(msg.event));

		if(it != this->cmd_angles_.end()) {
			point.x = this->radius_*cos(-it->second);
			point.y = this->radius_*sin(-it->second);
			point.z = 0.0f;
			channel.name = "strength";
			channel.values.clear();
			channel.values.push_back(-1.0f);
			this->data_.points.push_back(point);
			this->data_.channels.push_back(channel);
			this->data_.header.stamp = ros::Time::now();
		} else {
			ROS_WARN("%s cannot find a value for the received event: %d", this->GetName().c_str(), msg.event);
		}
	}

}

void DiscreteControl::Reset(void) {

	geometry_msgs::Point32 		point;
	sensor_msgs::ChannelFloat32 channel;

	this->data_.header.frame_id = "base_link";
	this->data_.points.clear();
	this->data_.channels.clear();
}

bool DiscreteControl::on_service_reset_(cnbiros_services::Reset::Request& req,
							   			cnbiros_services::Reset::Response& res) {
	res.result = false;
	if(req.reset == true) {
		ROS_INFO("%s has been requested to reset", this->GetName().c_str());
		
		if(this->IsStopped() == false) {
			this->Reset();
			this->Publish(this->topic_, this->data_);
			res.result = true;
		}
	}

	return res.result;

}

void DiscreteControl::onRunning(void) {
	if(this->data_.points.empty() == false)
		this->Publish(this->topic_, this->data_);
}
	
	}
}
#endif
