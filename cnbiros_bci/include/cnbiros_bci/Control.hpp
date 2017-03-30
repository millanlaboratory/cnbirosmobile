#ifndef CNBIROS_BCI_CONTROL_HPP
#define CNBIROS_BCI_CONTROL_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_bci/TiCMessage.h"
#include "cnbiros_bci/TiDMessage.h"


namespace cnbiros {
	namespace bci {

class Control : public core::RosInterface {

	public:
		Control(unsigned int type, std::string name);
		~Control(void);

		void AddSource(std::string topic);
		
		void SetFilterId(std::string pipe, int family = -1);
		void SetFilterIc(std::string pipe, std::string classifier, std::string label);
		
		void onRunning(void);
	private:
		void onReceivedTiC(const cnbiros_bci::TiCMessage& msg);
		void onReceivedTiD(const cnbiros_bci::TiDMessage& msg);

	public:
		static const unsigned int AsContinous = 1;
		static const unsigned int AsDiscrete  = 2;

	private:
		unsigned int type_;
		std::string  topic_;

		std::string	 filter_pipe_;
		std::string  filter_classifier_;
		std::string  filter_label_;
		int 		 filter_family_;

};

	}
}




#endif
