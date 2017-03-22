#ifndef CNBIROS_BCI_TIDPROXY_CPP
#define CNBIROS_BCI_TIDPROXY_CPP

#include "TiDProxy.hpp"

namespace cnbiros {
	namespace bci {

TiDProxy::TiDProxy(std::string name) : RosInterface(name) {
	this->SetPublisher<cnbiros_bci::TiDMessage>("/" + this->GetName());
}

TiDProxy::~TiDProxy(void) {
}

void TiDProxy::Attach(std::string pipe) {
	
	std::map<std::string, ClTobiId*>::iterator it;

	// Check if a ClTobiId is already attached to this pipe, and in case, delete it.
	it = this->tobiid_get_.find(pipe);
	if(it != this->tobiid_get_.end()) {
		it->second->Detach();
		delete it->second;
		this->tobiid_get_.erase(it);

		ROS_INFO("%s is already attached to %s as GetOnly. The old connection is replaced", 
				this->GetName().c_str(), pipe.c_str());
	}

	// Create a new ClTobiId and attach to the given pipe
	this->tobiid_get_[pipe] = new ClTobiId(ClTobiId::GetOnly);

	while(this->tobiid_get_[pipe]->IsAttached() == false) {
		this->tobiid_get_[pipe]->Attach(pipe);
		ROS_WARN_THROTTLE(5.0f, "%s waits for attaching to %s", this->GetName().c_str(), pipe.c_str());
		ros::Rate(1.0f).sleep();
	}

	ROS_INFO("%s attached to %s as GetOnly", this->GetName().c_str(), pipe.c_str());
}

void TiDProxy::Attach(std::string pipe, std::string topic) {

	std::map<std::string, ClTobiId*>::iterator it;
	
	// Check if a ClTobiId is already attached to this pipe, and in case, delete it.
	it = this->tobiid_set_.find(pipe);
	if(it != this->tobiid_set_.end()) {
		it->second->Detach();
		delete it->second;
		this->tobiid_set_.erase(it);

		ROS_INFO("%s is already attached to %s as SetOnly. The old connection is replaced", 
				this->GetName().c_str(), pipe.c_str());
	}

	this->tobiid_set_[pipe] = new ClTobiId(ClTobiId::SetOnly);
	
	while(this->tobiid_set_[pipe]->IsAttached() == false) {
		this->tobiid_set_[pipe]->Attach(pipe);
		ROS_WARN_THROTTLE(5.0f, "%s waits for attaching to %s", this->GetName().c_str(), pipe.c_str());
		ros::Rate(1.0f).sleep();
	}

	ROS_INFO("%s attached to %s as SetOnly", this->GetName().c_str(), pipe.c_str());

	this->tobiid_table_[pipe] = topic;
	this->SetSubscriber(topic, &TiDProxy::on_message_received_, this);
}

void TiDProxy::Detach(std::string pipe) {

	std::map<std::string, ClTobiId*>::iterator its;
	std::map<std::string, ClTobiId*>::iterator itg;

	its = this->tobiid_set_.find(pipe);

	if(its != this->tobiid_set_.end()) {
		if(its->second->IsAttached()) {
			its->second->Detach();
			this->tobiid_set_.erase(its);
			ROS_INFO("%s detached from %s (SetOnly)", this->GetName().c_str(), pipe.c_str());
		}
	} 	

	itg = this->tobiid_get_.find(pipe);

	if(itg != this->tobiid_get_.end()) {
		if(itg->second->IsAttached()) {
			itg->second->Detach();
			this->tobiid_get_.erase(itg);
			ROS_INFO("%s detached from %s (GetOnly)", this->GetName().c_str(), pipe.c_str());
		}
	} 
}

void TiDProxy::on_message_received_(const cnbiros_bci::TiDMessage& msg) {

	IDMessage idm;
	std::map<std::string, ClTobiId*>::iterator it;
	int fidx = TCBlock::BlockIdxUnset;

	it = this->tobiid_set_.find(msg.pipe);

	if (it != this->tobiid_set_.end()) {
		idm = this->ConvertTo(msg);	
		IDSerializerRapid ids(&idm);
		it->second->SetMessage(&ids, TCBlock::BlockIdxUnset, &fidx); 	
	} else {
		ROS_WARN("Message received for pipe \'%s\'. However such a pipe is not registered", msg.pipe.c_str());
	}
}


IDMessage TiDProxy::ConvertTo(const cnbiros_bci::TiDMessage& msg) {
	
	IDMessage idm;

	idm.SetFamilyType(msg.family);
	idm.SetDescription(msg.description);
	idm.SetEvent(msg.event);
	idm.SetBlockIdx();

	return idm;
}

cnbiros_bci::TiDMessage TiDProxy::ConvertFrom(const IDMessage& idm) {

	cnbiros_bci::TiDMessage msg;

	msg.header.stamp = ros::Time::now();
	msg.family 		 = idm.GetFamilyType();
	msg.description  = idm.GetDescription();
	msg.event 		 = idm.GetEvent();

	return msg;
}

void TiDProxy::onRunning(void) {

	IDMessage idm;
	IDSerializerRapid ids(&idm);
	std::map<std::string, ClTobiId*>::iterator it;
	cnbiros_bci::TiDMessage msg;

	for (it = this->tobiid_get_.begin(); it != this->tobiid_get_.end(); ++it) {

		if(it->second->GetMessage(&ids) == true) {
			msg = this->ConvertFrom(idm);
			msg.pipe = it->first;
			this->Publish("/" + this->GetName(), msg);
		}
	}


}


	}
}


#endif
