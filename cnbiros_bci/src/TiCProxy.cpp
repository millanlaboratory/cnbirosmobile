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



void TiCProxy::Detach(std::string pipe) {

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

void TiCProxy::on_rosmessage_received_(const cnbiros_bci::TiCMessage& msg) {
	
	ICMessage icm;
	ICSerializerRapid* ics;
	TobiIcTools* ictools;
	std::map<std::string, ClTobiIc*>::iterator it;
	bool is_attached = true;

	it = this->tobiic_set_.find(msg.pipe);

	// Check if the interface identified by the pipe name in the message is
	// attached. If not, it tries to attach.
	if(it->second->IsAttached() == false) {
		if(it->second->Attach(it->first) == false) {
			ROS_WARN_THROTTLE(5, "%s still not attached to %s. Ros message is not forwarded to CNBI loop.", 
								 this->GetName().c_str(), it->first.c_str());
			is_attached = false;
		} 
	}

	// If the target interface is attached, convert the ros tobiic message
	// received and send it to the CNBI loop through the target interface 
	if(is_attached == true) {
		if (it != this->tobiic_set_.end()) {

			ictools = new TobiIcTools(msg);

			ictools->GetMessage(icm);
			
			ics = new ICSerializerRapid(&icm);

			it->second->SetMessage(ics);
		}

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


	// Try to attach to reader interfaces (if they are not attached)
	for (it = this->tobiic_get_.begin(); it != this->tobiic_get_.end(); ++it) {
		if(it->second->IsAttached() == false) {
			if(it->second->Attach(it->first) == false) {
				ROS_WARN_THROTTLE(5, "%s still not attached to %s", 
							this->GetName().c_str(), it->first.c_str());
			} else {
				ROS_INFO("%s attached to %s as Reader", this->GetName().c_str(), it->first.c_str());
			}
		}
	}
	
	// Try to attach to writer interfaces (if they are not attached)
	for (it = this->tobiic_set_.begin(); it != this->tobiic_set_.end(); ++it) {
		if(it->second->IsAttached() == false) {
			if(it->second->Attach(it->first) == false) {
				ROS_WARN_THROTTLE(5, "%s still not attached to %s", 
							this->GetName().c_str(), it->first.c_str());
			} else {
				ROS_INFO("%s attached to %s as Writer", this->GetName().c_str(), it->first.c_str());
			}
		}
	}

	// For each reader interface that is attached, check if has message, convert
	// it and publish on topic
	for (it = this->tobiic_get_.begin(); it != this->tobiic_get_.end(); ++it) {
		
		if(it->second->IsAttached() == true) {
			if(it->second->GetMessage(&ics) == ClTobiIc::HasMessage) {
				ROS_INFO_ONCE("Streaming started from pipe %s", it->first.c_str());
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
