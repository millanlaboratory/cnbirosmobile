#ifndef CNBIROS_FORCEFIELD_ACTORS_CPP
#define CNBIROS_FORCEFIELD_ACTORS_CPP

#include "cnbiros_forcefield/Actors.hpp"

namespace cnbiros {
	namespace forcefield {

Actors::Actors(std::string name) : Fusion(name) {
}

Actors::~Actors(void){};

void Actors::Process(void) {
	
	this->grid_.Sum(this->GetName());	
	this->grid_.SetMinMax(this->GetName(), -1.0f, 1.0f);
}


	}
}

#endif
