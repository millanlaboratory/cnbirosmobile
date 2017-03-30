#ifndef CNBIROS_FORCEFIELD_ACTORS_CPP
#define CNBIROS_FORCEFIELD_ACTORS_CPP

#include "cnbiros_forcefield/Actors.hpp"

namespace cnbiros {
	namespace forcefield {

Actors::Actors(const unsigned int type, std::string name) : Fusion(name) {
	this->type_ = type;

	switch(this->type_) {
		case Actors::AsRepellor:
			this->sign_ = 1;
			break;
		case Actors::AsAttractor:
			this->sign_ = -1;
			break;
	}
}

Actors::~Actors(void){};

unsigned int Actors::GetType(void) {
	return this->type_;
}

void Actors::Process(void) {
	
	//this->grid_.Reset(this->GetName());
	this->grid_.Sum(this->GetName());	
	
	this->grid_[this->GetName()] *= this->sign_;
	this->grid_.SetMinMax(this->GetName(), -1.0f, 1.0f);
	

}


	}
}

#endif
