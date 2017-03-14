#ifndef CNBIROS_ROBOTINO_COM_HPP
#define CNBIROS_ROBOTINO_COM_HPP

#include <rec/robotino/api2/Com.h>
#include <ros/ros.h>


namespace cnbiros {
	namespace robotino {

class RobotinoCom : public rec::robotino::api2::Com {

	public:
		RobotinoCom(std::string owner);

		void Connect(std::string address);
		void Disconnect(void);
		bool IsConnected(void);

		void errorEvent(const char* errorString);
		void connectedEvent(void);
		void connectionClosedEvent(void);

	private:
		std::string owner_;
};

	}
}

#endif
