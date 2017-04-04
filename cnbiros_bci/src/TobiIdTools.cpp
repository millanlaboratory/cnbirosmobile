#ifndef CNBIROS_BCI_TOBIID_TOOLS_CPP
#define CNBIROS_BCI_TOBIID_TOOLS_CPP

#include "cnbiros_bci/TobiIdTools.hpp"

namespace cnbiros {
	namespace bci {

TobiIdTools::TobiIdTools(const cnbiros_bci::TiDMessage& msg) {
	this->rosmsg_ = msg;
}

TobiIdTools::TobiIdTools(const IDMessage& idm) {

	// Fill the ros message with the IDMessage. Using default frameid
	this->rosmsg_.header.stamp 		= ros::Time::now();
	this->rosmsg_.header.frame_id 	= "base_link";
	this->rosmsg_.family 		 	= idm.GetFamilyType();
	this->rosmsg_.description  		= idm.GetDescription();
	this->rosmsg_.event 		 	= idm.GetEvent();
}

TobiIdTools::~TobiIdTools(void) {}

void TobiIdTools::GetMessage(IDMessage& idm) {

	// Fill the IDMessage with the ros message fields. Since ors has not a frame
	// block (as the cnbiloop), set the block id as unset.
	idm.SetFamilyType(this->rosmsg_.family);
	idm.SetDescription(this->rosmsg_.description);
	idm.SetEvent(this->rosmsg_.event);
	idm.SetBlockIdx(TCBlock::BlockIdxUnset);
}

void TobiIdTools::GetMessage(cnbiros_bci::TiDMessage& out) {
	out = this->rosmsg_;
}

bool TobiIdTools::IsFromPipe(const std::string& pipe) {
	bool is_valid = false;

	if(this->rosmsg_.pipe.compare(pipe) == 0) {
		is_valid = true;
	}

	return is_valid;
}


	}
}

#endif
