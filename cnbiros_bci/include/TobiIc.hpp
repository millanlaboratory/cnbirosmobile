#ifndef CNBIROS_BCI_TOBIIC_HPP
#define CNBIROS_BCI_TOBIIC_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiIc.hpp>
#include <tobiic/ICClassifier.hpp>

#include "RosInterface.hpp"
#include "TobiIcTools.hpp"
#include "cnbiros_bci/TobiIc.h"

namespace cnbiros {
	namespace bci {

class TobiIc : public core::RosInterface {
	
	public:
		TobiIc(void);
		virtual ~TobiIc(void);
	
		void Attach(std::string pipe);
		void Attach(std::string pipe, std::string topic);
		void Detach(std::string pipe);

		void ConvertTo(ICMessage* icm, const cnbiros_bci::TobiIc& msg);
		//cnbiros_bci::TobiIc ConvertFrom(const ICMessage& idm);

	private:
		void onRunning(void);
		virtual void on_message_received_(const cnbiros_bci::TobiIc& msg);

	private:
		std::map<std::string, ClTobiIc*> 	tobiic_get_;
		std::map<std::string, ClTobiIc*> 	tobiic_set_;
		std::map<std::string, std::string> 	tobiic_table_;

		std::vector<ICClassifier> 			icclassifiers_;
		std::vector<ICClass>				icclasses_;


};


	}
}


#endif
