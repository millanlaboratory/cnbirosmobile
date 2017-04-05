#ifndef CNBIROS_FORCEFIELD_DYNAMIC_HPP
#define CNBIROS_FORCEFIELD_DYNAMIC_HPP

#include "cnbiros_core/Flags.hpp"
#include "cnbiros_core/Navigation.hpp"

namespace cnbiros {
	namespace forcefield {

class Dynamics : public core::Navigation {
	
	public:
		Dynamics(std::string name = "navigation");
		~Dynamics(void);

		void onRunning(void);

	protected:
		bool AngularVelocity(geometry_msgs::Twist& msg);
		void LinearVelocity(geometry_msgs::Twist& msg);

	private:
		float compute_angle_(float x, float y);
		float compute_distance_(float x, float y);
		float compute_lambda_(float distance, float beta1, float beta2);
		float compute_sigma_(float distance, float obstruction);


	private:
		std::string 	layer_;


};


	}
}

#endif
