/* TO RE-THINK ABOUT 
#ifndef SENSORFACTORY_CPP
#define SENSORFACTORY_CPP

#include "SensorFactory.hpp"

namespace cnbiros {
	namespace core {

SensorFactory::SensorFactory(void){};
SensorFactory::~SensorFactory(void){};

Sensor* SensorFactory::CreateSensor(Sensor::Type type) {
	switch(type) {
		case Sensor::Type::Infrared:
			return new Infrared;
		//case Sensor::Type::Sonar:
		//	return new Sonar;
		//case Sensor::Type::Kinect:
		//	return new Kinect;
		//case Sensor::Type::Camera:
		//	return new Camera;
		default:
			return nullptr;
	}
}


	}
}

#endif
*/
