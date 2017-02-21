#include "Robotino.hpp"

#define ROBOTINO_RATE 	10 // Hz
#define ROBOTINO_IP "192.168.1.3"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	Robotino robotino(ROBOTINO_IP, ROBOTINO_RATE);

	robotino.Dump();

	sleep(1);

	auto u = 0;
	while(robotino.IsConnected() & u < 3) {
	
		robotino.Run();
		sleep(1);
		u++;
	};

	robotino.Run();
	sleep(3);

	return 0;

}
