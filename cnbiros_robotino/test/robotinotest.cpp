#include "RobotinoBase.hpp"

using namespace cnbiros::core;
using namespace cnbiros::robotino;

int main(int argc, char** argv) {

	RobotinoBase robotino("192.168.1.3", 10.0f);

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
