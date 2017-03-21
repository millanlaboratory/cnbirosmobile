#ifndef CNBIROS_BCI_TOBIIC_CPP
#define CNBIROS_BCI_TOBIIC_CPP

#include "TobiIc.hpp"

namespace cnbiros {
	namespace bci {

TobiIc::TobiIc(void) : RosInterface("tobiic") {
	this->SetPublisher<cnbiros_bci::TobiIc>("/tobiic");
}

TobiIc::~TobiIc(void) {
}

void TobiIc::Attach(std::string pipe) {
	
	std::map<std::string, ClTobiIc*>::iterator it;

	// Check if a ClTobiIc is already attached to this pipe, and in case, delete it.
	it = this->tobiic_get_.find(pipe);
	if(it != this->tobiic_get_.end()) {
		it->second->Detach();
		delete it->second;
		this->tobiic_get_.erase(it);

		ROS_INFO("%s is already attached to %s as GetOnly. The old connection is replaced", 
				this->GetName().c_str(), pipe.c_str());
	}

	// Create a new ClTobiId and attach to the given pipe
	this->tobiic_get_[pipe] = new ClTobiIc(ClTobiIc::GetOnly);

	while(this->tobiic_get_[pipe]->IsAttached() == false) {
		this->tobiic_get_[pipe]->Attach(pipe);
		ROS_WARN_THROTTLE(5.0f, "%s waits for attaching to %s", this->GetName().c_str(), pipe.c_str());
		ros::Rate(1.0f).sleep();
	}

	ROS_INFO("%s attached to %s as GetOnly", this->GetName().c_str(), pipe.c_str());
}

void TobiIc::Attach(std::string pipe, std::string topic) {

	std::map<std::string, ClTobiIc*>::iterator it;
	
	// Check if a ClTobiIc is already attached to this pipe, and in case, delete it.
	it = this->tobiic_set_.find(pipe);
	if(it != this->tobiic_set_.end()) {
		it->second->Detach();
		delete it->second;
		this->tobiic_set_.erase(it);

		ROS_INFO("%s is already attached to %s as SetOnly. The old connection is replaced", 
				this->GetName().c_str(), pipe.c_str());
	}

	this->tobiic_set_[pipe] = new ClTobiIc(ClTobiIc::SetOnly);
	
	while(this->tobiic_set_[pipe]->IsAttached() == false) {
		this->tobiic_set_[pipe]->Attach(pipe);
		ROS_WARN_THROTTLE(5.0f, "%s waits for attaching to %s", this->GetName().c_str(), pipe.c_str());
		ros::Rate(1.0f).sleep();
	}

	ROS_INFO("%s attached to %s as SetOnly", this->GetName().c_str(), pipe.c_str());

	this->tobiic_table_[pipe] = topic;
	this->SetSubscriber(topic, &TobiIc::on_message_received_, this);
}

void TobiIc::Detach(std::string pipe) {

	std::map<std::string, ClTobiIc*>::iterator its;
	std::map<std::string, ClTobiIc*>::iterator itg;

	its = this->tobiic_set_.find(pipe);

	if(its != this->tobiic_set_.end()) {
		if(its->second->IsAttached()) {
			its->second->Detach();
			this->tobiic_set_.erase(its);
			ROS_INFO("%s detached from %s (SetOnly)", this->GetName().c_str(), pipe.c_str());
		}
	} 	

	itg = this->tobiic_get_.find(pipe);

	if(itg != this->tobiic_get_.end()) {
		if(itg->second->IsAttached()) {
			itg->second->Detach();
			this->tobiic_get_.erase(itg);
			ROS_INFO("%s detached from %s (GetOnly)", this->GetName().c_str(), pipe.c_str());
		}
	} 
}

void TobiIc::on_message_received_(const cnbiros_bci::TobiIc& msg) {
	
	ICMessage icm;
	ICSerializerRapid* ics;
	TobiIcTools* ictools;
	std::map<std::string, ClTobiIc*>::iterator it;

	it = this->tobiic_set_.find(msg.pipe);
	
	if (it != this->tobiic_set_.end()) {

		ictools = new TobiIcTools(msg);

		ictools->GetMessage(&icm);
		
		ics = new ICSerializerRapid(&icm);

		it->second->SetMessage(ics);
	}

	delete ics;
	delete ictools;
}

void TobiIc::ConvertTo(ICMessage* icm, const cnbiros_bci::TobiIc& msg) {
	
	//std::vector<cnbiros_bci::TobiIcClassifier>::const_iterator itclf;
	//std::vector<cnbiros_bci::TobiIcClass>::const_iterator itcls;

	//std::vector<ICClassifier> icclassifiers;

	//ICClass k1("0x300", 0.60f);
	////ICClassifier c("c1", "test", 0, 0);
	////c.classes.Add(&k1);
	////icm->classifiers.Add(&c);
	//for(itclf = msg.IcClassifiers.begin(); itclf != msg.IcClassifiers.end(); ++itclf) {
	//	ICClassifier c((*itclf).name, (*itclf).description, (*itclf).vtype, (*itclf).ltype);
	//	icclassifiers.push_back(c);

	//	auto u = &icclassifiers.back();
	//	u->classes.Add(&k1);
	//	for(itcls = (*itclf).IcClasses.begin(); itcls != (*itclf).IcClasses.end(); ++itcls) {
	//		this->icclasses_.push_back(ICClass((*itcls).label, (*itcls).value));
	//		auto i = &(*(this->icclasses_.end()-1));
	//		u->classes.Add(i);
	//	}

	//}
	//for(auto i = icclassifiers.begin(); i != icclassifiers.end(); ++i) {
	//	printf("%s: %s (%d, %d)\n", (*i).GetName().c_str(), (*i).GetDescription().c_str(), 
	//								(*i).GetValueType(), (*i).GetLabelType());

	//	icm->classifiers.Add(&(*i));
	//}

	//printf("In Message:\n");
	//for(auto u = icm->classifiers.Begin(); u != icm->classifiers.End(); ++u) {
	//	printf("%s:\n", u->second->GetName().c_str());
	//}
	//icm->SetBlockIdx(1);
	//icm->Dump();
	//printf("after dump\n");
	////return icm;
}

/*
cnbiros_bci::TobiId TobiId::ConvertFrom(const IDMessage& idm) {

	cnbiros_bci::TobiId msg;

	msg.header.stamp = ros::Time::now();
	msg.family 		 = idm.GetFamilyType();
	msg.description  = idm.GetDescription();
	msg.event 		 = idm.GetEvent();

	return msg;
}

*/
void TobiIc::onRunning(void) {

	ICMessage icm;
	ICSerializerRapid ics(&icm);
	std::map<std::string, ClTobiIc*>::iterator it;
	cnbiros_bci::TobiIc msg;
	TobiIcTools* ictools;

	for (it = this->tobiic_get_.begin(); it != this->tobiic_get_.end(); ++it) {

		if(it->second->GetMessage(&ics) == ClTobiIc::HasMessage) {
			ROS_INFO("Message received from %s: ", it->first.c_str());
			ictools = new TobiIcTools(icm);
			ictools->GetMessage(msg);


			for(auto c = msg.classifiers.begin(); c != msg.classifiers.end(); ++c) {
				printf("classifier: %s\n", (*c).name.c_str());
				
				for(auto i = (*c).classes.begin(); i != (*c).classes.end(); ++i) { 
					printf("|- label: %s, value: %f\n", (*i).label.c_str(), (*i).value);
				}
			}




			msg.pipe = it->first;
			this->Publish("/tobiic", msg);
		}
	}


}

	}
}


#endif
