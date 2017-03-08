#ifndef CNBIROS_CORE_ROSINTERFACE_CPP
#define CNBIROS_CORE_ROSINTERFACE_CPP

#include "RosInterface.hpp"

namespace cnbiros {
	namespace core {

RosInterface::RosInterface(ros::NodeHandle* node) {
	this->rosnode_ 	  		= node;
	this->name_    	  		= "rosinterface";
	this->is_stopped_ 		= false;
	this->frequency_  		= CNBIROS_NODE_FREQUENCY;
	this->rosrate_    		= new ros::Rate(this->frequency_);
	this->rosframe_child_  	= "";
	this->rosframe_parent_ 	= "";


	// Initialize services
	this->rossrv_state_ = this->rosnode_->advertiseService("rosinterface_state",
									&RosInterface::on_rosinterface_service_, this); 
	
}

RosInterface::~RosInterface(void) {
	this->rosnode_->shutdown();
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

void RosInterface::SetName(std::string name) {
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

	while(this->rosnode_->ok()) {

		if(this->IsStopped() == false) {
			
			this->onRunning();

			this->SendTransform(this->rostf_msg_);
		}

		rosrate_->sleep();
		ros::spinOnce();
	}

	ros::spin();
}

void RosInterface::onRunning(void) {}

void RosInterface::SetParentFrame(std::string frameid) {
	this->rosframe_parent_ = frameid;
}

std::string RosInterface::GetParentFrame(void) {
	return this->rosframe_parent_;
}

void RosInterface::SetChildFrame(std::string frameid) {
	this->rosframe_child_ = frameid;
}

std::string RosInterface::GetChildFrame(void) {
	return this->rosframe_child_;
}


geometry_msgs::TransformStamped RosInterface::GetTransformMessage(void) {
	return this->rostf_msg_;
}


void RosInterface::SetTransformMessage(tf::Vector3 translation, float yaw) {
	this->rostf_msg_.header.stamp 			 = ros::Time::now();
	this->rostf_msg_.header.frame_id 		 = this->rosframe_parent_;
	this->rostf_msg_.child_frame_id  		 = this->rosframe_child_;
	this->rostf_msg_.transform.translation.x = translation.x();
	this->rostf_msg_.transform.translation.y = translation.y();
	this->rostf_msg_.transform.translation.z = translation.z();
	this->rostf_msg_.transform.rotation 	 = tf::createQuaternionMsgFromYaw(yaw);
}

void RosInterface::SetTransformMessage(tf::Vector3 translation, geometry_msgs::Quaternion quaternion) {
	this->rostf_msg_.header.stamp 			 = ros::Time::now();
	this->rostf_msg_.header.frame_id 		 = this->rosframe_parent_;
	this->rostf_msg_.child_frame_id  		 = this->rosframe_child_;
	this->rostf_msg_.transform.translation.x = translation.x();
	this->rostf_msg_.transform.translation.y = translation.y();
	this->rostf_msg_.transform.translation.z = translation.z();
	this->rostf_msg_.transform.rotation 	 = quaternion;
}

void RosInterface::SendTransform(geometry_msgs::TransformStamped msg) {

	if(this->rosframe_child_.empty() == false && this->rosframe_parent_.empty() == false) {
		this->rostf_broadcaster_.sendTransform(msg);
	}
}

void RosInterface::TransformPoint(std::string frame, 
								  geometry_msgs::PointStamped& in,
								  geometry_msgs::PointStamped& out) {
	
	try {
		this->rostf_listener_.transformPoint(frame, in, out);
	} catch(tf::TransformException& ex) {
		ROS_WARN("Exception trying to transform a point from %s"
				  " to %s: %s", in.header.frame_id.c_str(), frame.c_str(), ex.what());
	}
}

	}
}


#endif
