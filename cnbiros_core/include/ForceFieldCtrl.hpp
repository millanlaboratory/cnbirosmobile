#ifndef FORCEFIELDCTRL_HPP
#define FORCEFIELDCTRL_HPP

#include "NavigationCtrl.hpp"

namespace cnbiros {
	namespace core {

class ForceFieldCtrl : public NavigationCtrl {
	
	public:
		ForceFieldCtrl(float frequency = CNBIROS_CTRL_NODE_FREQUENCY);
		~ForceFieldCtrl(void);

		void Run(void);

	protected:
		void AngularVelocity(void);
		void LinearVelocity(void);

};


	}
}

#endif
