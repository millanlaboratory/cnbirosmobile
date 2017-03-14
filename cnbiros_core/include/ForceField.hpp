#ifndef CNBIROS_CORE_FORCEFIELD_HPP
#define CNBIROS_CORE_FORCEFIELD_HPP

#include "Flags.hpp"
#include "Navigation.hpp"

namespace cnbiros {
	namespace core {

class ForceField : public Navigation {
	
	public:
		ForceField(std::string name=CNBIROS_FORCEFIELD_NAME);
		~ForceField(void);

		void SetGridLayer(std::string layer);
		std::string GetGridLayer(void);

		void onRunning(void);

	protected:
		bool AngularVelocity(geometry_msgs::Twist& msg);
		void LinearVelocity(geometry_msgs::Twist& msg);

	private:
		float compute_angle_(float x, float y);
		float compute_distance_(float x, float y);
		float compute_lambda_(float distance, float beta1, float beta2);
		float compute_sigma_(float distance, float obstruction);


};


	}
}

#endif
