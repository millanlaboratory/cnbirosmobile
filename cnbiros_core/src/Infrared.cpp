#ifndef INFRARED_CPP
#define INFRARED_CPP

#include "Infrared.hpp"

namespace cnbiros {
	namespace core {

Infrared::Infrared(std::string name) : Sensor(name) {
	this->nsensors_ = 0;
}

void Infrared::SetNumSensors(unsigned int nsensors) {
	this->nsensors_ = nsensors;
}

unsigned int Infrared::GetNumSensors(void) {
	return this->nsensors_;
}

	}
}


#endif
