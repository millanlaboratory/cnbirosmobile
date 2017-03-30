#ifndef CNBIROS_BCI_CONTROL_CPP
#define CNBIROS_BCI_CONTROL_CPP

#include "cnbiros_bci/Control.hpp"

namespace cnbiros {
	namespace bci {

Control::Control(unsigned int type, std::string name) : RosInterface(name) {
	this->type_  = type;
	this->topic_ = "/" + this->GetName();

	this->filter_pipe_ 		 = "";
	this->filter_classifier_ = "";
	this->filter_label_ 	 = "";
	this->filter_family_ 	 = -1;
	this->SetPublisher<geometry_msgs::Point32>(this->topic_);
}

Control::~Control(void) {}

void Control::AddSource(std::string topic) {
	
	switch(this->type_) {
		case Control::AsContinous:
			this->SetSubscriber(topic, &Control::onReceivedTiC, this);
			break;
		case Control::AsDiscrete:
			this->SetSubscriber(topic, &Control::onReceivedTiD, this);
			break;
	}
}

void Control::SetFilterId(std::string pipe, int family) {
	this->filter_pipe_   = pipe;
	this->filter_family_ = family;
}

void Control::SetFilterIc(std::string pipe, std::string classifier, std::string label) {
	this->filter_pipe_   	 = pipe;
	this->filter_classifier_ = classifier;
	this->filter_label_ 	 = label;
}

void Control::onReceivedTiC(const cnbiros_bci::TiCMessage& msg) {
}

void Control::onReceivedTiD(const cnbiros_bci::TiDMessage& msg) {

	geometry_msgs::Point32 point;
	bool check_pipe   = true;
	bool check_family = true;
	float radius = 0.3f;
	float angle;
	
	printf("Received ID message\n");
	if(this->filter_pipe_.empty() == false) {
		if(this->filter_pipe_.compare(msg.pipe) != 0 ) {
			check_pipe = false;
		}
	}

	if(this->filter_family_ != -1) {
		if(this->filter_family_ != msg.family) {
			check_family = false;
		}
	}

	if(check_pipe && check_family) {

		switch(msg.event) {
			case 1:
				angle = -M_PI/4;
				break;
			case 2:
				angle = M_PI/4;
				break;
		}

		point.x = radius*cos(angle);
		point.y = radius*sin(angle);
		point.z = 0.0f;

		this->Publish(this->topic_, point);
	}

}

void Control::onRunning(void) {
}
	
	}
}
#endif
