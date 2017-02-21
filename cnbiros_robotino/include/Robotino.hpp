#ifndef ROBOTINO_HPP
#define ROBOTINO_HPP

#include <rec/robotino/api2/OmniDrive.h>
#include "Robot.hpp"
#include "RobotinoCom.hpp"


namespace cnbiros {
	namespace robotino {

class Robotino : public core::Robot {
	public:	
		Robotino(std::string hostname, unsigned int frequency, 
				 std::string name="robotino", 
				 std::string id="00000001");
		~Robotino(void);
		void Run (void);

		bool IsConnected(void);


		virtual void velocityCallback(const cnbiros_messages::RobotVelocity& msg);

	protected:
		RobotinoCom* com_;
		float vx_;
		float vy_;
		float vo_;
		rec::robotino::api2::OmniDrive omnidrive_;
};



	}
}



#endif
