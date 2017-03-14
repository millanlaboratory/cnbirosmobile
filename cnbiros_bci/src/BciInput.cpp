#ifndef CNBIROS_BCI_BCIINPUT_CPP
#define CNBIROS_BCI_BCIINPUT_CPP

#include "BciInput.hpp"

namespace cnbiros {
	namespace bci {

BciInput::BciInput(void) {
	this->SetName("bci");

	this->SetSubscriber("/tobiid_bci2ros", &BciInput::on_tobiid_received_, this);
	this->SetSubscriber("/tobiic_bci2ros", &BciInput::on_tobiic_received_, this);
	this->SetSubscriber("/odom", &BciInput::on_odometry_received_, this);
	
	
	this->rosgrid_.add("bci_discrete");
	this->SetGrid("bci_discrete", 5.0f, 5.0f, 0.05f);
	
	this->rosgrid_.add("bci_continous");
	this->SetGrid("bci_continous", 5.0f, 5.0f, 0.05f);

	// Initialization TF
	this->SetParentFrame("odom");
	this->SetChildFrame("base_bci");
	this->SetTransformMessage(tf::Vector3(0.0f, 0.0f, 0.0f), 0.0f);
	
	this->target_angle_ = 0.0f;
}

BciInput::~BciInput(void) {}

void BciInput::AdvertiseOn(std::string topic) {
	this->SetPublisher<grid_map_msgs::GridMap>(topic);
}

void BciInput::SetGrid(std::string layer, float xsize, float ysize, float res,
					 std::string frame) {

	if(this->rosgrid_.exists(layer)) {
		core::GridMapTool::SetGeometry(this->rosgrid_, xsize, ysize, res);
		core::GridMapTool::SetFrameId(this->rosgrid_, frame);
		core::GridMapTool::Reset(this->rosgrid_, layer);
	}
}

void BciInput::on_odometry_received_(const nav_msgs::Odometry& msg) {

	this->rosodom_msg_ = msg;

	float orientation = this->GetOrientation(this->rosodom_msg_);

	//printf("Target: %f, Orientation: %f, Diff: %f\n", this->target_angle_, 
	//		orientation, fabs(orientation - this->target_angle_));

	if (this->target_angle_ != 0.0f) {
		if(fabs(orientation - this->target_angle_) < 0.05f) {
			core::GridMapTool::Reset(this->rosgrid_, "bci_discrete");
			this->target_angle_ = 0.0f;
			ROS_INFO("TARGET REACHED");
		}
	}

}

float BciInput::GetOrientation(nav_msgs::Odometry& msg) {
  	tf::Pose pose;
  	tf::poseMsgToTF(msg.pose.pose, pose);
  	return tf::getYaw(pose.getRotation());
}

void BciInput::on_tobiid_received_(const cnbiros_messages::TobiId& msg) {

	float angle, x, y, radius;
	radius = 0.1f;

	tf::Quaternion quaternion(this->rosodom_msg_.pose.pose.orientation.x,
							  this->rosodom_msg_.pose.pose.orientation.y,
							  this->rosodom_msg_.pose.pose.orientation.z,
							  this->rosodom_msg_.pose.pose.orientation.w);
	tf::Matrix3x3 m(quaternion);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	geometry_msgs::PointStamped base_bci, base_link;
	base_bci.header.frame_id = this->GetChildFrame();
	
	core::GridMapTool::Reset(this->rosgrid_, "bci_discrete");

	switch(msg.event) {
		case 1:
			angle = M_PI/4.0f; // Left
			break;
		case 2:
			angle = -M_PI/4.0f; // Right
			break;
		default:
			angle = 0.0f;
			break;
	}

	base_bci.point.x = (radius)*cos(angle);
	base_bci.point.y = (radius)*sin(angle);
	
	// Transform point from kinect frame to base frame
	this->TransformPoint(this->GetParentFrame(), base_bci, base_link);

	// Convert x,y cohordinates in position
	grid_map::Position position(base_link.point.x, base_link.point.y);
	
	// Skip if positions outside the grid range
	if(this->rosgrid_.isInside(position)) {
		this->rosgrid_.atPosition("bci_discrete", position) = -1.0f;
		this->received_time_ = ros::Time::now();
		this->target_angle_ = this->GetOrientation(this->rosodom_msg_) - angle;
		printf("Target at %f\n", this->target_angle_);
	}

}

void BciInput::ResetDiscrete(float time) {

	float elapsed;
	
	elapsed = ros::Time::now().toSec() - this->received_time_.toSec();
	
	if(elapsed > time) {
		core::GridMapTool::Reset(this->rosgrid_, "bci_discrete");
	}

}


void BciInput::onRunning(void) {

	grid_map_msgs::GridMap msg;
	

	// Publish the grid map	
	core::GridMapTool::ToMessage(this->rosgrid_, msg);
	this->Publish(msg);

	//this->ResetDiscrete(3.0f);
}

	}
}


#endif
