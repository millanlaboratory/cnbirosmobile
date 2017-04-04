#ifndef CNBIROS_BCI_TIDPROXY_CPP
#define CNBIROS_BCI_TIDPROXY_CPP

#include "cnbiros_bci/TiDProxy.hpp"

namespace cnbiros {
	namespace bci {

TiDProxy::TiDProxy(std::string name) : RosInterface(name) {
	this->SetPublisher<cnbiros_bci::TiDMessage>("/" + this->GetName());
}

TiDProxy::~TiDProxy(void) {

	// Destruction of the map and its own pointers
	for(auto it = this->id_map_.begin(); it != this->id_map_.end(); ++it) {
		delete it->second;
		this->id_map_.erase(it);
	}
}

bool TiDProxy::Attach(unsigned int mode, std::string pipe, std::string topic) {

	bool status;

	// Detach from the pipe (if exists)
	if(this->Detach(pipe)) {
		ROS_WARN("%s detached from %s (already exists)", this->GetName().c_str(), pipe.c_str());
	}

	// According to the modality create the specific interface 
	switch(mode) {
		case TiDProxy::AsReader:
			this->id_map_[pipe] = new ClTobiId(ClTobiId::GetOnly);
			this->pm_map_[pipe] = TiDProxy::AsReader;
			break;
		case TiDProxy::AsWriter:
			this->id_map_[pipe] = new ClTobiId(ClTobiId::SetOnly);
			this->pm_map_[pipe] = TiDProxy::AsWriter;
		
		case TiDProxy::AsReaderWriter:
			this->id_map_[pipe] = new ClTobiId(ClTobiId::SetGet);
			this->pm_map_[pipe] = TiDProxy::AsReaderWriter;
			break;
	}

	// If the modality is AsWriter or AsReaderWriter and the topic is not empty,
	// set a subscriber for this topic
	if (mode == TiDProxy::AsWriter || mode == TiDProxy::AsReaderWriter) {
		if(topic.empty() == false) {
			this->pt_map_[pipe] = topic;
			this->SetSubscriber(topic, &TiDProxy::onReceived, this);
		} else {
			ROS_WARN("Topic is empty. %s does not subscribe to any topic", this->GetName().c_str());
		}
	}

	// Try to attach to the pipe
	status = this->id_map_[pipe]->Attach(pipe);

	// Return the status of the connection
	return status;
}

bool TiDProxy::IsAttached(std::string pipe) {

	bool result = false;
	std::map<std::string, ClTobiId*>::iterator it;

	// Look for an interface registered to the given pipe
	it = this->id_map_.find(pipe);

	// If exists, check if the interface is attached
	if(it != this->id_map_.end()) {
		result = it->second->IsAttached();
	}

	return result;
}

bool TiDProxy::Detach(std::string pipe) {

	bool result = false;
	std::map<std::string, ClTobiId*>::iterator it;
	std::map<std::string, std::string>::iterator itpt;
	std::map<std::string, unsigned int>::iterator itpm;

	// Look for an interface registered to the given pipe
	it = this->id_map_.find(pipe);

	// If the interface has been found, detach it and delete from the list.
	// If it also exists in the pipe_topic_map, delete it from there.
	// If it also exists in the pipe_mode_map, delete it frome there.
	if(it != this->id_map_.end()) {
		if(it->second->IsAttached()) {
			it->second->Detach();
			this->id_map_.erase(it);
			
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

bool TiDProxy::IsWriter(std::string pipe) {

	std::map<std::string, unsigned int>::iterator it;

	it = this->pm_map_.find(pipe);
	return (it->second == TiDProxy::AsWriter || it->second == TiDProxy::AsReaderWriter);
}


bool TiDProxy::IsReader(std::string pipe) {

	std::map<std::string, unsigned int>::iterator it;

	it = this->pm_map_.find(pipe);
	return (it->second == TiDProxy::AsReader || it->second == TiDProxy::AsReaderWriter);
}

bool TiDProxy::IsReaderWriter(std::string pipe) {

	std::map<std::string, unsigned int>::iterator it;

	it = this->pm_map_.find(pipe);
	return (it->second == TiDProxy::AsReaderWriter);
}

void TiDProxy::onReceived(const cnbiros_bci::TiDMessage& msg) {
	
	IDMessage idm;
	IDSerializerRapid ids(&idm);
	std::map<std::string, ClTobiId*>::iterator it;
	TobiIdTools* tools;

	std::map<std::string, std::string>::iterator itpt;
	std::map<std::string, unsigned int>::iterator itpm;
	bool status = true;

	// Check if the interface defined in msg.pipe is registered and attached. If
	// not, try to Attach
	if(this->IsAttached(msg.pipe) == false) {
		ROS_WARN_THROTTLE(5, "%s is not attached to %s", this->GetName().c_str(), msg.pipe.c_str()); 
		status = false;
	
		// Retrieve topic for this pipe and the stored modality for this
		// interface
		itpt = this->pt_map_.find(msg.pipe);
		itpm = this->pm_map_.find(msg.pipe);

		if(itpt != this->pt_map_.end() && itpm != this->pm_map_.end()) {
			status = this->Attach(itpm->second, msg.pipe, itpt->second);
		} else {
			ROS_ERROR("%s can't find a topic or a modality registered for %s", 
					  this->GetName().c_str(), msg.pipe.c_str());
		}

		if(status == true) {
			ROS_INFO("%s attached to %s", this->GetName().c_str(), msg.pipe.c_str());	
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
		it = this->id_map_.find(msg.pipe);
		tools = new TobiIdTools(msg);
		tools->GetMessage(idm);
		it->second->SetMessage(&ids);

		delete tools;
	}
}

void TiDProxy::onRunning(void) {

	IDMessage idm;
	IDSerializerRapid ids(&idm);
	std::map<std::string, ClTobiId*>::iterator it;
	std::map<std::string, unsigned int>::iterator itm;
	std::map<std::string, std::string>::iterator itp;
	cnbiros_bci::TiDMessage msg;
	TobiIdTools* tools;

	// Check for all the reader or readerwriter interface registered and not attached. If it
	// finds them, it tries to attach.
	for (it = this->id_map_.begin(); it != this->id_map_.end(); ++it) {
		if(this->IsReader(it->first) && (this->IsAttached(it->first) == false)) {

			// Find modality for the given pipe
			itm = this->pm_map_.find(it->first);

			// Find topic (if ReaderWriter modality)
			std::string topic = "";
			if(itm->second == TiDProxy::AsReaderWriter) {
				itp = this->pt_map_.find(it->first);
				if (itp != this->pt_map_.end()) {
					topic = itp->second;
				}
			}

			if(this->Attach(itm->second, it->first, topic) == false) {
				ROS_WARN_THROTTLE(5, "%s is not attached to %s", this->GetName().c_str(), it->first.c_str());
			} else {
				ROS_INFO("%s attached to %s", this->GetName().c_str(), it->first.c_str());	
			}
		}
	}

	// For each reader interface that is attached, check if has message, convert
	// it and publish on topic
	for (it = this->id_map_.begin(); it != this->id_map_.end(); ++it) {
		if(this->IsReader(it->first) && this->IsAttached(it->first)) {
			if(it->second->GetMessage(&ids) == true) {
				tools = new TobiIdTools(idm);
				tools->GetMessage(msg);
				msg.pipe = it->first;
				this->Publish("/" + this->GetName(), msg);
				delete tools;
			}
		} 
	}

}

	}
}


#endif
