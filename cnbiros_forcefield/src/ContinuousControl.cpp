#ifndef CNBIROS_FORCEFIELD_CONTINUOUS_CPP
#define CNBIROS_FORCEFIELD_CONTINUOUS_CPP

#include "cnbiros_forcefield/ContinuousControl.hpp"

namespace cnbiros {
	namespace forcefield {

ContinuousControl::ContinuousControl(std::string name) : RosInterface(name) {
	this->topic_ = "/" + this->GetName();
	this->flt_is_set_ = false;
	this->flt_pipe_   = "";
	this->flt_name_   = "";
	this->flt_label_  = "";

	this->SetPublisher<sensor_msgs::PointCloud>(this->topic_);

	// Service for sensor reset
	this->rossrv_reset_ = this->advertiseService("reset_continuous_control", 
											&ContinuousControl::on_service_reset_, this);
	
	this->Reset();	
}

ContinuousControl::~ContinuousControl(void) {}

void ContinuousControl::AddSource(std::string topic) {
	this->SetSubscriber(topic, &ContinuousControl::onReceived, this);
}

void ContinuousControl::SetRadius(float radius) {
	this->radius_ = radius;
}

void ContinuousControl::SetFilter(std::string pipe, std::string name, std::string label) {
	this->flt_pipe_   = pipe;
	this->flt_name_   = name;
	this->flt_label_  = label;
	this->flt_is_set_ = true;
}

bool ContinuousControl::CheckMessage(const cnbiros_bci::TiCMessage& msg) {

	cnbiros::bci::TobiIcTools tools(msg);

	bool is_valid = true;

	if(this->flt_is_set_ == true) {
		is_valid = tools.IsFromPipe(this->flt_pipe_) && tools.HasClass(this->flt_name_, this->flt_label_);
	}

	return is_valid;
}

void ContinuousControl::onReceived(const cnbiros_bci::TiCMessage& msg) {

	geometry_msgs::Point32 		point;
	sensor_msgs::ChannelFloat32 channel;
	float angle;
	cnbiros_bci::TiCClass icclass;
	cnbiros::bci::TobiIcTools tools(msg);

	if(this->CheckMessage(msg) == true) {
		this->Reset();
		
		if(tools.GetClass(this->flt_name_, this->flt_label_, icclass)) {
			angle = icclass.value;
			point.x = this->radius_*cos(-angle);
			point.y = this->radius_*sin(-angle);
			point.z = 0.0f;
			channel.name = "strength";
			channel.values.clear();
			channel.values.push_back(-1.0f);
			this->data_.points.push_back(point);
			this->data_.channels.push_back(channel);
			this->data_.header.stamp = ros::Time::now();

		} else {
			ROS_WARN("%s cannot find a value in this classifier %s and with this label: %s", 
					  this->GetName().c_str(), this->flt_name_.c_str(), this->flt_label_.c_str());
		}

	}
}

void ContinuousControl::Reset(void) {

	geometry_msgs::Point32 		point;
	sensor_msgs::ChannelFloat32 channel;

	this->data_.header.frame_id = "base_link";
	this->data_.points.clear();
	this->data_.channels.clear();
}

bool ContinuousControl::on_service_reset_(cnbiros_services::Reset::Request& req,
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

void ContinuousControl::onRunning(void) {
	if(this->data_.points.empty() == false)
		this->Publish(this->topic_, this->data_);
}


	}
}

#endif


