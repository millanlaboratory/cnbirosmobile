#ifndef CNBIROS_CORE_FORCEFIELD_HPP
#define CNBIROS_CORE_FORCEFIELD_HPP

#include "Navigation.hpp"

namespace cnbiros {
	namespace core {

class ForceField : public Navigation {
	
	public:
		ForceField(ros::NodeHandle* node);
		~ForceField(void);

		void Run(void);

	protected:
		void AngularVelocity(void);
		void LinearVelocity(void);

};


	}
}

#endif
