#ifndef CNBIROS_BCI_TICPROXY_CPP
#define CNBIROS_BCI_TICPROXY_CPP

#include "TiCProxy.hpp"

namespace cnbiros {
	namespace bci {

TiCProxy::TiCProxy(std::string name) : RosInterface(name) {
	this->SetPublisher<cnbiros_bci::TiCMessage>("/" + this->GetName());
}

TiCProxy::~TiCProxy(void) {
}

bool TiCProxy::Attach(unsigned int mode, std::string pipe, std::string topic) {

	bool status;

	// Detach from the pipe (if exists)
	if(this->Detach(pipe)) {
		ROS_WARN("%s detached from %s (already exists)", this->GetName().c_str(), pipe.c_str());
	}

	// According to the modality create the specific interface 
	switch(mode) {
		case TiCProxy::AsReader:
			this->ic_map_[pipe] = new ClTobiIc(ClTobiIc::GetOnly);
			this->pm_map_[pipe] = TiCProxy::AsReader;
			break;
		case TiCProxy::AsWriter:
			this->ic_map_[pipe] = new ClTobiIc(ClTobiIc::SetOnly);
			this->pm_map_[pipe] = TiCProxy::AsWriter;

			// If the topic is not empty, set a subscriber for this topic
			if(topic.empty() == false) {
				this->pt_map_[pipe] = topic;
				this->SetSubscriber(topic, &TiCProxy::onReceived, this);
			} else {
				ROS_WARN("Topic is empty. %s does not subscribe to any topic", this->GetName().c_str());
			}
			break;
	}

	// Try to attach to the pipe
	status = this->ic_map_[pipe]->Attach(pipe);

	// Return the status of the connection
	return status;
}

bool TiCProxy::IsAttached(std::string pipe) {

	bool result = false;
	std::map<std::string, ClTobiIc*>::iterator it;

	// Look for an interface registered to the given pipe
	it = this->ic_map_.find(pipe);

	// If exists, check if the interface is attached
	if(it != this->ic_map_.end()) {
		result = it->second->IsAttached();
	}

	return result;
}

bool TiCProxy::Detach(std::string pipe) {

	bool result = false;
	std::map<std::string, ClTobiIc*>::iterator it;
	std::map<std::string, std::string>::iterator itpt;
	std::map<std::string, unsigned int>::iterator itpm;

	// Look for an interface registered to the given pipe
	it = this->ic_map_.find(pipe);

	// If the interface has been found, detach it and delete from the list.
	// If it also exists in the pipe_topic_map, delete it from there.
	// If it also exists in the pipe_mode_map, delete it frome there.
	if(it != this->ic_map_.end()) {
		if(it->second->IsAttached()) {
			it->second->Detach();
			this->ic_map_.erase(it);
			
			itpt = this->pt_map_.find(pipe);
			if(itpt != this->pt_map_.end())
				this->pt_map_.erase(itpt);

			itpm = this->pm_map_.find(pipe);
			if(itpm != this->pm_map_.end())
				this->pm_map_.erase(itpm);
			
			result = true;
		}
	} 	

	return result;
}

bool TiCProxy::IsWriter(std::string pipe) {

	std::map<std::string, unsigned int>::iterator it;

	it = this->pm_map_.find(pipe);
	return (it->second == TiCProxy::AsWriter);
}

bool TiCProxy::IsReader(std::string pipe) {

	std::map<std::string, unsigned int>::iterator it;

	it = this->pm_map_.find(pipe);
	return (it->second == TiCProxy::AsReader);
}

/*
bool TiCProxy::Attach(std::string pipe) {
	
	std::map<std::string, ClTobiIc*>::iterator it;

	// Check if a ClTobiIc is already attached to this pipe, and in case, delete it.
	it = this->tobiic_get_.find(pipe);
	if(it != this->tobiic_get_.end()) {
		if(it->second->IsAttached()) {
			it->second->Detach();
		}
		delete it->second;
		this->tobiic_get_.erase(it);

		ROS_INFO("%s is already attached to %s as Reader. The old connection is replaced", 
				this->GetName().c_str(), pipe.c_str());
	}

	// Create a new ClTobiIc and attach to the given pipe
	this->tobiic_get_[pipe] = new ClTobiIc(ClTobiIc::GetOnly);
	
	if(this->tobiic_get_[pipe]->Attach(pipe) == false) {
		ROS_WARN("%s pipe not ready. %s not attached yet.", pipe.c_str(), this->GetName().c_str());
	} else {
		ROS_INFO("%s attached to %s as Reader", this->GetName().c_str(), pipe.c_str());
	}

	return this->tobiic_get_[pipe]->IsAttached();
}

bool TiCProxy::Attach(std::string pipe, std::string topic) {

	std::map<std::string, ClTobiIc*>::iterator it;
	
	// Check if a ClTobiIc is already attached to this pipe, and in case, delete it.
	it = this->tobiic_set_.find(pipe);
	if(it != this->tobiic_set_.end()) {
		if(it->second->IsAttached()) {
			it->second->Detach();
		}
		delete it->second;
		this->tobiic_set_.erase(it);

		ROS_INFO("%s is already registered to %s as Reader. The old connection is replaced", 
				this->GetName().c_str(), pipe.c_str());
	}

	// Create a new tobiic as writer
	this->tobiic_set_[pipe] = new ClTobiIc(ClTobiIc::SetOnly);

	// Try to attach
	if(this->tobiic_set_[pipe]->Attach(pipe) == false) {
		ROS_WARN("%s pipe not ready. %s not attached yet.", pipe.c_str(), this->GetName().c_str());
	} else {
		ROS_INFO("%s attached to %s as Writer", this->GetName().c_str(), pipe.c_str());
	}

	// Set subscriber for the writer interface
	this->SetSubscriber(topic, &TiCProxy::on_rosmessage_received_, this);

	return this->tobiic_set_[pipe]->IsAttached();
}



void TiCProxy::Detach2(std::string pipe) {

	std::map<std::string, ClTobiIc*>::iterator its;
	std::map<std::string, ClTobiIc*>::iterator itg;

	its = this->tobiic_set_.find(pipe);

	if(its != this->tobiic_set_.end()) {
		if(its->second->IsAttached()) {
			its->second->Detach();
			this->tobiic_set_.erase(its);
			ROS_INFO("%s detached from %s (Writer)", this->GetName().c_str(), pipe.c_str());
		}
	} 	

	itg = this->tobiic_get_.find(pipe);

	if(itg != this->tobiic_get_.end()) {
		if(itg->second->IsAttached()) {
			itg->second->Detach();
			this->tobiic_get_.erase(itg);
			ROS_INFO("%s detached from %s (Reader)", this->GetName().c_str(), pipe.c_str());
		}
	} 
}
*/
void TiCProxy::onReceived(const cnbiros_bci::TiCMessage& msg) {
	
	ICMessage icm;
	ICSerializerRapid* ics;
	TobiIcTools* ictools;
	std::map<std::string, ClTobiIc*>::iterator it;

	std::map<std::string, std::string>::iterator itpt;
	bool status = true;


	// Check if the interface defined in msg.pipe is registered and attached. If
	// not, try to Attach
	if(this->IsAttached(msg.pipe) == false) {
		ROS_WARN_THROTTLE(5, "%s is not attached to %s", this->GetName().c_str(), msg.pipe.c_str()); 
		status = false;
	
		// Retrieve topic for this pipe
		itpt = this->pt_map_.find(msg.pipe);
		if(itpt != this->pt_map_.end()) {
			status = this->Attach(TiCProxy::AsWriter, msg.pipe, itpt->second);
		} else {
			ROS_ERROR("%s can't find a topic registered on %s", this->GetName().c_str(), msg.pipe.c_str());
		}

		if(status == true) {
			ROS_INFO("%s attached to %s as writer", this->GetName().c_str(), msg.pipe.c_str());	
		}
	}

	// Check if the interface defined in msg.pipe is set as writer
	if (status == true) {
		if(this->IsWriter(msg.pipe) == false) {
			status = false;
			ROS_ERROR("%s is connected to %s as reader", this->GetName().c_str(), msg.pipe.c_str());
		}
	}

	// If everything is fine (status == true), then convert the received
	// rosmessage and forward it to the loop
	if(status == true) {

		ictools = new TobiIcTools(msg);
		ictools->GetMessage(icm);
		ics = new ICSerializerRapid(&icm);

		it = this->ic_map_.find(msg.pipe);
		it->second->SetMessage(ics);

		delete ics;
		delete ictools;
	}
}

void TiCProxy::onRunning(void) {

	ICMessage icm;
	ICSerializerRapid ics(&icm);
	std::map<std::string, ClTobiIc*>::iterator it;
	cnbiros_bci::TiCMessage msg;
	TobiIcTools* ictools;

	// Check for all the reader interface registered and not attached. If it
	// finds them, it tries to attach.
	for (it = this->ic_map_.begin(); it != this->ic_map_.end(); ++it) {
		if(this->IsReader(it->first) && (this->IsAttached(it->first) == false)) {
			if(this->Attach(TiCProxy::AsReader, it->first) == false) {
				ROS_WARN_THROTTLE(5, "%s is not attached to %s", this->GetName().c_str(), it->first.c_str());
			} else {
				ROS_INFO("%s attached to %s as reader", this->GetName().c_str(), it->first.c_str());	
			}
		}
	}

	// For each reader interface that is attached, check if has message, convert
	// it and publish on topic
	for (it = this->ic_map_.begin(); it != this->ic_map_.end(); ++it) {
		if(this->IsReader(it->first) && this->IsAttached(it->first)) {
			if(it->second->GetMessage(&ics) == ClTobiIc::HasMessage) {
				ROS_INFO_ONCE("%s started streaming on topic /%s", this->GetName().c_str(), this->GetName().c_str());
				ictools = new TobiIcTools(icm);
				ictools->GetMessage(msg);

				msg.pipe = it->first;
				this->Publish("/" + this->GetName(), msg);

				delete ictools;
			}
		} 
	}

}

	}
}


#endif
