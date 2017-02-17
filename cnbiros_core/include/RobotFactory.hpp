/* TO RE-THINK ABOUT 

#ifndef ROBOTFACTORY_HPP
#define ROBOTFACTORY_HPP

#include <memory>
#include "Robot.hpp"

namespace cnbiros {
	namespace core {

class RobotFactory {
	public:
		Robot* Get(void);
		virtual void CreateRobot(Robot::Type type) = 0;

	protected:
		std::unique_ptr<Robot> robot_;
};


	}
}

#endif
*/
