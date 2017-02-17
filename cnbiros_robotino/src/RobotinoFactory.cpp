/* TO RE-THINK ABOUT 
#ifndef ROBOTINOFACTORY_CPP
#define ROBOTINOFACTORY_CPP

#include "RobotinoFactory.hpp"

namespace cnbiros {
	namespace robotino {

void RobotinoFactory::CreateRobot(core::Robot::Type type) {

	switch(type) {
		case core::Robot::Type::Robotino:
			//this->robot_ = std::make_unique<Robotino>(); 			// C++14
			this->robot_ = std::unique_ptr<Robotino>(new Robotino);	// C++11
			break;
		default:
			// error handling
			break;
	}
}

	}
}

#endif
*/
