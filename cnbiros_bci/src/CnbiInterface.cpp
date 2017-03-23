#ifndef CNBIROS_BCI_CNBIINTERFACE_CPP
#define CNBIROS_BCI_CNBIINTERFACE_CPP

#include "CnbiInterface.hpp"

namespace cnbiros {
	namespace bci {

CnbiInterface::CnbiInterface(const CcAddress address) {

	// Initialize the loop
	this->cnbiaddress_ = address;

	if(address.empty() == false) {
		ClLoop::Configure(address);
	} else {
		ClLoop::Configure();
	}
}

CnbiInterface::~CnbiInterface(void) {
	CcCore::Exit(0);
};

bool CnbiInterface::Connect(bool wait) {

	do {
		if(ClLoop::Connect() == false)
			ROS_WARN_THROTTLE(5, "Wait to connect to cnbiloop at %s", cnbiaddress_.c_str());
	} while(ClLoop::IsConnected() == false && wait == true);

	ROS_INFO("Connected to cnbiloop at %s", cnbiaddress_.c_str());
}

void CnbiInterface::Disconnect(void) {
	ClLoop::Disconnect();
}

std::string CnbiInterface::GetAddress(void) {
	return this->cnbiaddress_;
}

	}
}


#endif
