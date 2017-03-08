#ifndef CNBIROS_BCI_TOBIID_HPP
#define CNBIROS_BCI_TOBIID_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiId.hpp>

#include "cnbiros_messages/TobiId.h"
#include "RosInterface.hpp"

namespace cnbiros {
	namespace bci {

class TobiId : public core::RosInterface {
	
	public:
		TobiId(ros::NodeHandle* node, unsigned int mode);
		virtual ~TobiId(void);
	
		bool Attach(std::string pipe, float wait = 10.0f);
		void Detach(void);
		bool IsAttached(void);

		void SetMessage(std::string description, unsigned int family);

		void ConvertToIdMessage(cnbiros_messages::TobiId* rosmsg, IDMessage* idm);
		void ConvertFromIdMessage(IDMessage* idm, cnbiros_messages::TobiId* rosmsg);

		void onRunning(void);
	protected:
		virtual void on_tobiid_received_(const cnbiros_messages::TobiId& msg);

	protected:
		ClTobiId* 			tobiid_;
		IDMessage*			idm_;
		IDSerializerRapid* 	ids_;


};


	}
}


#endif
