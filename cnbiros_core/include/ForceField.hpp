#ifndef CNBIROS_CORE_FORCEFIELD_HPP
#define CNBIROS_CORE_FORCEFIELD_HPP

#define CNBIROS_FORCEFIELD_INFLUENCE 		1.0f		// Maximum radius of influences 			[meters]
#define CNBIROS_FORCEFIELD_OBSTRUCTION 		0.4f		// Default obstruction (size of the device) [meters] 
#define CNBIROS_FORCEFIELD_STRENGTH 		1.0f		// Default stength of attractors/repellors
#define CNBIROS_FORCEFIELD_SPATIALDECAY 	0.5f		// Default spatial decay of attractors/repellors

#include "Navigation.hpp"

namespace cnbiros {
	namespace core {

class ForceField : public Navigation {
	
	public:
		ForceField(ros::NodeHandle* node);
		~ForceField(void);

		void SetGridLayer(std::string layer);
		std::string GetGridLayer(void);

		void SetInfluence(float radius);
		void SetObstruction(float size);
		void SetStrength(float strength);
		void SetSpatialDecay(float decay);

		float GetInfluence(void);
		float GetObstruction(void);
		float GetStrength(void);
		float GetSpatialDecay(void);

		void onRunning(void);

	protected:
		void AngularVelocity(void);
		void LinearVelocity(void);

	private:
		float ToAngle(float x, float y);
		float ToRadius(float x, float y);
		float GetLambda(float distance, float beta1, float beta2);
		float GetSigma(float distance, float obstruction);

	private:
		float 	influence_;
		float 	obstruction_;
		float 	strength_;
		float 	spatialdecay_;

		std::string rosgrid_layer_;

};


	}
}

#endif
