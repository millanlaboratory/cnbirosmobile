#ifndef CNBIROS_FORCEFIELD_ACTORS_HPP
#define CNBIROS_FORCEFIELD_ACTORS_HPP


#include <ros/ros.h>
#include "cnbiros_core/SensorGrid.hpp"
#include "cnbiros_core/Fusion.hpp"

namespace cnbiros {
	namespace forcefield {

class Actors : public core::Fusion {
	public:
		Actors(const unsigned int type, std::string name);
		~Actors(void);

		unsigned int GetType(void);

		void Process(void);

	public:
		static const unsigned int AsAttractor = 1;
		static const unsigned int AsRepellor  = 2;


	private:
		unsigned int type_;
		int 		 sign_;
};

	}
}


#endif
