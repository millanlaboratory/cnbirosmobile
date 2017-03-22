#ifndef CNBIROS_BCI_TIDPROXY_HPP
#define CNBIROS_BCI_TIDPROXY_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiId.hpp>

#include "cnbiros_bci/TiDMessage.h"
#include "RosInterface.hpp"

namespace cnbiros {
	namespace bci {

class TiDProxy : public core::RosInterface {
	
	public:
		TiDProxy(std::string name = "tidproxy");
		virtual ~TiDProxy(void);
	
		void Attach(std::string pipe);
		void Attach(std::string pipe, std::string topic);
		void Detach(std::string pipe);

		IDMessage ConvertTo(const cnbiros_bci::TiDMessage& rosmsg);
		cnbiros_bci::TiDMessage ConvertFrom(const IDMessage& idm);

		void onRunning(void);
	private:
		virtual void on_message_received_(const cnbiros_bci::TiDMessage& msg);

	private:
		std::map<std::string, ClTobiId*> 	tobiid_get_;
		std::map<std::string, ClTobiId*> 	tobiid_set_;
		std::map<std::string, std::string> 	tobiid_table_;


};


	}
}


#endif
