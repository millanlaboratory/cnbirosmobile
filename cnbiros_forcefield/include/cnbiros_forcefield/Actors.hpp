#ifndef CNBIROS_FORCEFIELD_ACTORS_HPP
#define CNBIROS_FORCEFIELD_ACTORS_HPP


#include <ros/ros.h>
#include "cnbiros_core/SensorGrid.hpp"
#include "cnbiros_core/Fusion.hpp"

namespace cnbiros {
	namespace forcefield {

class Actors : public core::Fusion {
	public:
		Actors(std::string name);
		~Actors(void);

		void Process(void);

};

	}
}


#endif
