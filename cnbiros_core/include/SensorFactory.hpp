#ifndef SENSORFACTORY_HPP
#define SENSORFACTORY_HPP

#include "Sensor.hpp"
#include "Infrared.hpp"


namespace cnbiros {
	namespace core {

class SensorFactory {
	
	public:
		SensorFactory(void);
		virtual ~SensorFactory(void);
		static Sensor* CreateSensor(Sensor::Type type);

};
	}
}

#endif
