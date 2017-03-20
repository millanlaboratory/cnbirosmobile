#ifndef CNBIROS_BCI_CNBIINTERFACE_HPP
#define CNBIROS_BCI_CNBIINTERFACE_HPP

#include <ros/ros.h>
#include <cnbiloop/ClLoop.hpp>

namespace cnbiros {
	namespace bci {

class CnbiInterface {

	public: 
		CnbiInterface(const CcAddress address = "");
		~CnbiInterface(void);

		bool Connect(bool wait = true);
		void Disconnect(void);

		// To do: handle cnbilog

	private:
		CcAddress cnbiaddress_;



};

	}
}

#endif
