#ifndef CNBIROS_CORE_CNBIINTERFACE_HPP
#define CNBIROS_CORE_CNBIINTERFACE_HPP

#include <ros/ros.h>
#include <cnbiloop/ClLoop.hpp>

#include "RosInterface.hpp"

namespace cnbiros {
	namespace core {

class CnbiInterface : public RosInterface {
	
	public:
		CnbiInterface(ros::NodeHandle* node);
		virtual ~CnbiInterface(void);
	
		void SetAddress(std::string address);
		bool Connect(void);

	protected:
		std::string address_;

};


	}
}


#endif
