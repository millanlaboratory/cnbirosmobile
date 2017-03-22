#ifndef CNBIROS_BCI_TICPROXY_HPP
#define CNBIROS_BCI_TICPROXY_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiIc.hpp>
#include <tobiic/ICClassifier.hpp>
#include <cnbicore/CcProxy.hpp>

#include "RosInterface.hpp"
#include "TobiIcTools.hpp"
#include "cnbiros_bci/TiCMessage.h"

namespace cnbiros {
	namespace bci {


class TiCProxy : public core::RosInterface {
	
	public:
		TiCProxy(std::string name = "ticproxy");
		virtual ~TiCProxy(void);
	
		bool Attach(std::string pipe);
		bool Attach(std::string pipe, std::string topic);
		void Detach(std::string pipe);

	private:
		void onRunning(void);
		virtual void on_rosmessage_received_(const cnbiros_bci::TiCMessage& msg);

	private:
		std::map<std::string, ClTobiIc*> 	tobiic_get_;
		std::map<std::string, ClTobiIc*> 	tobiic_set_;

		std::vector<ICClassifier> 			icclassifiers_;
		std::vector<ICClass>				icclasses_;

};


	}
}


#endif
