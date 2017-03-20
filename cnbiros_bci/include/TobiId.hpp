#ifndef CNBIROS_BCI_TOBIID_HPP
#define CNBIROS_BCI_TOBIID_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiId.hpp>

#include "cnbiros_bci/TobiId.h"
#include "RosInterface.hpp"

namespace cnbiros {
	namespace bci {

class TobiId : public core::RosInterface {
	
	public:
		TobiId(void);
		virtual ~TobiId(void);
	
		void Attach(std::string pipe);
		void Attach(std::string pipe, std::string topic);
		void Detach(std::string pipe);

		IDMessage ConvertTo(const cnbiros_bci::TobiId& rosmsg);
		cnbiros_bci::TobiId ConvertFrom(const IDMessage& idm);

		void onRunning(void);
	private:
		virtual void on_message_received_(const cnbiros_bci::TobiId& msg);

	private:
		std::map<std::string, ClTobiId*> 	tobiid_get_;
		std::map<std::string, ClTobiId*> 	tobiid_set_;
		std::map<std::string, std::string> 	tobiid_table_;


};


	}
}


#endif
