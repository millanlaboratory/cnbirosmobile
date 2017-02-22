#ifndef INFRARED_HPP
#define INFRARED_HPP

#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

class Infrared : public Sensor {

	public:
		Infrared(std::string name="infrared");
		virtual ~Infrared(void){};
	
		void SetNumSensors(unsigned int nsensors);
		unsigned int GetNumSensors(void);

		//virtual void Process(void) {};


	protected:
		unsigned int nsensors_;
};


	}
}


#endif
