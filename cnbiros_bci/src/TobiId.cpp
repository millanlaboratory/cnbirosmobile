#ifndef CNBIROS_BCI_TOBIID_CPP
#define CNBIROS_BCI_TOBIID_CPP

#include "TobiId.hpp"

namespace cnbiros {
	namespace bci {

TobiId::TobiId(ros::NodeHandle* node, unsigned int mode) : core::RosInterface(node) {
	this->SetName("tobiid");

	this->tobiid_ = new ClTobiId(mode);
	this->idm_ 	  = new IDMessage;
	this->ids_ 	  = new IDSerializerRapid(this->idm_);

	this->SetMessage("bci", IDMessage::FamilyBiosig);
	this->idm_->SetEvent(0);

	this->SetSubscriber("/tobiid_ros2bci", &TobiId::on_tobiid_received_, this);
	this->SetPublisher<cnbiros_messages::TobiId>("/tobiid_bci2ros");
}

TobiId::~TobiId(void) {
	this->Detach();
	delete this->tobiid_;
}

bool TobiId::Attach(std::string pipe, float wait) {

	ros::Time begin, current;
	ros::Rate rate(1);

	begin = ros::Time::now();
	while(this->IsAttached() == false) {
		
		this->tobiid_->Attach(pipe);
		
		current = ros::Time::now();	
		if (current.toSec() >= wait) {
			break;
		}
		rate.sleep();
	}

	if(this->IsAttached()) {
		ROS_INFO("%s attached to %s", this->GetName().c_str(), pipe.c_str());
	} else {
		ROS_ERROR("%s cannot attach to %s", this->GetName().c_str(), pipe.c_str());
	}

	return this->IsAttached();
}

void TobiId::Detach(void) {
	if(this->IsAttached())
		this->tobiid_->Detach();
}

bool TobiId::IsAttached(void) {
	return this->tobiid_->IsAttached();
}

void TobiId::SetMessage(std::string description, unsigned int family) {
	this->idm_->SetDescription(description);
	this->idm_->SetFamilyType(family);
}

void TobiId::on_tobiid_received_(const cnbiros_messages::TobiId& rosmsg) {
	
	cnbiros_messages::TobiId msg = rosmsg;

	this->ConvertToIdMessage(&msg, this->idm_);
	
	if(this->tobiid_->SetMessage(this->ids_))
		ROS_INFO("New message sent to bci");
}

void TobiId::ConvertToIdMessage(cnbiros_messages::TobiId* rosmsg, IDMessage* idm) {

	idm->SetFamilyType(rosmsg->family);
	idm->SetDescription(rosmsg->description);
	idm->SetEvent(rosmsg->event);
	idm->SetBlockIdx();
}

void TobiId::ConvertFromIdMessage(IDMessage* idm, cnbiros_messages::TobiId* rosmsg) {
	rosmsg->header.stamp = ros::Time::now();
	rosmsg->frame 		 = idm->GetBlockIdx();
	rosmsg->family 		 = idm->GetFamily();
	rosmsg->description  = idm->GetDescription();
	rosmsg->event 		 = idm->GetEvent();
}

void TobiId::onRunning(void) {

	cnbiros_messages::TobiId rosmsg;


	if(this->tobiid_->GetMessage(this->ids_) == true) {
	
		ROS_INFO("New ID message received from bci");
		this->ConvertFromIdMessage(this->idm_, &rosmsg);
		this->Publish(rosmsg);
	}


}


	}
}


#endif
