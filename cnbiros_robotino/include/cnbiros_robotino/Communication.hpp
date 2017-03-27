#ifndef CNBIROS_ROBOTINO_COMMUNICATION_HPP
#define CNBIROS_ROBOTINO_COMMUNICATION_HPP

#include <rec/robotino/api2/Com.h>
#include <ros/ros.h>

namespace cnbiros {
	namespace robotino {

class Communication : public rec::robotino::api2::Com {
	
	public:
		Communication(std::string owner);
		~Communication(void);

		void Connect(std::string address, bool reconnect = true);
		void Disconnect(void);
		bool IsConnected(void);

		std::string GetAddress(void);
		void SetAddress(std::string address);

		void errorEvent(const char* errorString);
		void connectedEvent(void);
		void connectionClosedEvent(void);

	private:
		std::string owner_;
		std::string address_;
};

	}
}

#endif
