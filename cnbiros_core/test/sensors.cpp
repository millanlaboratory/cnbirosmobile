#include "SensorFactory.hpp"
#include "Infrared.hpp"

using namespace cnbiros::core;

int main(int argc, char** argv) {

	SensorFactory factory;
	Infrared* infrared;
	unsigned int ninfrared;

	infrared = dynamic_cast<Infrared*>(factory.CreateSensor(Sensor::Type::Infrared));

	infrared->SetNumSensors(8);
	ninfrared = infrared->GetNumSensors();

	printf("Number of sensors for infrared: %d\n", ninfrared);



	return 0;
}
