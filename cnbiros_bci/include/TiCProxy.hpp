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

		bool Attach(unsigned int mode, std::string pipe, std::string topic = "");
		bool Detach(std::string pipe);

		bool IsAttached(std::string pipe);
		bool IsWriter(std::string pipe);
		bool IsReader(std::string pipe);

	private:
		void onRunning(void);
		void onReceived(const cnbiros_bci::TiCMessage& msg);

	public:
		const static unsigned int AsReader = 0;
		const static unsigned int AsWriter = 1;

	private:
		std::map<std::string, ClTobiIc*> 	ic_map_;
		std::map<std::string, std::string>	pt_map_;
		std::map<std::string, unsigned int> pm_map_;


};


	}
}


#endif
