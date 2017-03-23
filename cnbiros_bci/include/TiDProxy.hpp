#ifndef CNBIROS_BCI_TIDPROXY_HPP
#define CNBIROS_BCI_TIDPROXY_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiId.hpp>

#include "RosInterface.hpp"
#include "TobiIdTools.hpp"
#include "cnbiros_bci/TiDMessage.h"

namespace cnbiros {
	namespace bci {

class TiDProxy : public core::RosInterface {
	
	public:
		TiDProxy(std::string name = "tidproxy");
		virtual ~TiDProxy(void);
	
		bool Attach(unsigned int mode, std::string pipe, std::string topic = "");
		bool Detach(std::string pipe);

		bool IsAttached(std::string pipe);
		bool IsWriter(std::string pipe);
		bool IsReader(std::string pipe);
		bool IsReaderWriter(std::string pipe);
		
	private:
		void onRunning(void);
		void onReceived(const cnbiros_bci::TiDMessage& msg);

	public:
		const static unsigned int AsReader = 0;
		const static unsigned int AsWriter = 1;
		const static unsigned int AsReaderWriter = 2;

	private:
		std::map<std::string, ClTobiId*> 	id_map_;
		std::map<std::string, std::string>	pt_map_;
		std::map<std::string, unsigned int> pm_map_;



};


	}
}


#endif
