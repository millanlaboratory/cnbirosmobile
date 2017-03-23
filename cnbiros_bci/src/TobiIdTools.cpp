#ifndef CNBIROS_BCI_TOBIID_TOOLS_CPP
#define CNBIROS_BCI_TOBIID_TOOLS_CPP

#include "TobiIdTools.hpp"

namespace cnbiros {
	namespace bci {

TobiIdTools::TobiIdTools(void) {}
TobiIdTools::~TobiIdTools(void) {}

void TobiIdTools::GetMessage(const cnbiros_bci::TiDMessage& in, IDMessage& out) {

	// Fill the IDMessage with the ros message fields. Since ors has not a frame
	// block (as the cnbiloop), set the block id as unset.
	out.SetFamilyType(in.family);
	out.SetDescription(in.description);
	out.SetEvent(in.event);
	out.SetBlockIdx(TCBlock::BlockIdxUnset);
}

void TobiIdTools::GetMessage(const IDMessage& in, cnbiros_bci::TiDMessage& out) {

	// Fill the ros message with the IDMessage. Using default frameid
	out.header.stamp 	= ros::Time::now();
	out.header.frame_id = "id_link";
	out.family 		 	= in.GetFamilyType();
	out.description  	= in.GetDescription();
	out.event 		 	= in.GetEvent();
}


	}
}

#endif
