#ifndef CNBIROS_CORE_CNBIINTERFACE_CPP
#define CNBIROS_CORE_CNBIINTERFACE_CPP

#include "CnbiInterface.hpp"

namespace cnbiros {
	namespace core {

CnbiInterface::CnbiInterface(ros::NodeHandle* node) {
	this->Register(node);	
	this->SetName("cnbiinterface");
}

CnbiInterface::~CnbiInterface(void) {}

void CnbiInterface::SetAddress(std::string address) {
	this->address_ = address;
}

bool CnbiInterface::Connect(void) {
	ClLoop::Configure(this->address_);
	return ClLoop::Connect();
}

	}
}


#endif
