#include <cnbiloop/ClLoop.hpp>
#include <cnbiloop/ClTobiIc.hpp>
#include <getopt.h>
#include <cmath>
void usage(void) { 
	printf("Usage: cl_ticsender [OPTION]...\n\n");
	printf("  -a       address of nameserver in ip:port format\n");
	printf("  -p       TCP port (9500 default)\n");
	printf("  -n       TCP client name (/ctrl0 default)\n");
	printf("  -h       display this help and exit\n");
}

float sinewave(double time, double frequency) {
	return sin((2*M_PI)*time*frequency/1000.0f);
}


int main(int argc, char* argv[]) {
	int opt;
	std::string optname("/ctrl0");
	CcPort optport("");
	CcAddress nameserver;
	bool locking = false, dump = false;
	
	while((opt = getopt(argc, argv, "a:p:n:h")) != -1) {
		if(opt == 'p')
			optport.assign(optarg);
		else if(opt == 'n')
			optname.assign(optarg);
		else if(opt == 'd')
			dump = true;
		else if(opt == 'a')
			nameserver.assign(optarg);
		else {
			usage();
			CcCore::Exit(opt == 'h' ? EXIT_SUCCESS : EXIT_FAILURE);
		}
	}

	CcCore::OpenLogger("cl_ticsender");
	CcCore::CatchSIGINT();
	CcCore::CatchSIGTERM();
	ClLoop::Configure(nameserver);
	ICMessage icm;
	ICSerializerRapid ics(&icm);

	ICClass k1("0x301", 0.5f);
	ICClass k2("0x302", 0.5f);
	ICClass k3("0x303", 0.5f);
	ICClass k4("0x304", 0.5f);

	ICClassifier c1("classifier1", "Sinusoid classifier at 10 Hz",
					ICClassifier::ValueProb, ICClassifier::LabelCustom);
	ICClassifier c2("classifier2", "Sinusoid classifier at 20 Hz", 
					ICClassifier::ValueProb, ICClassifier::LabelCustom);

	c1.classes.Add(&k1);
	c1.classes.Add(&k2);

	c2.classes.Add(&k3);
	c2.classes.Add(&k4);

	icm.classifiers.Add(&c1);
	icm.classifiers.Add(&c2);

	icm.Dump();
	printf("a\n");	
	ClTobiIc ic(ClTobiIc::SetOnly);
	std::string absolute, relative;
	bool attached;
	int status;
	
	if(ClLoop::Connect() == false) {
		CcLogFatal("Cannot connect to loop");
		CcCore::Exit(EXIT_FAILURE);
	}
	
	attached = false;
	CcLogInfoS("Waiting for connection at " << optname <<":"<<optport);
	while(attached == false) {
		attached = optport.empty() ? ic.Attach(optname) : ic.Attach(optport, optname);
		CcTime::Sleep(1000.0f);
	}
	CcLogInfoS("Connected at " << optname <<":"<<optport);


	int block = 0;
	float value1, value2;

	CcTimeValue start;
	CcTime::Tic(&start);
	while(true) {
		icm.SetBlockIdx(block++);

		value1 = sinewave(CcTime::Toc(&start), 10);
		value2 = sinewave(CcTime::Toc(&start), 20);

		icm.SetValue("classifier1", "0x301", value1);
		icm.SetValue("classifier1", "0x302", 1 - value1);

		icm.SetValue("classifier2", "0x303", value2);
		icm.SetValue("classifier2", "0x304", 1 - value2);
		
		CcTime::Sleep(50.0f);

		if(ic.SetMessage(&ics) == ClTobiIc::Detached) {
			CcLogFatal("Tobi Ic detached");
			break;
		}

		if(CcCore::receivedSIGAny.Get()) 
			break;
	}


	ic.Detach();
	CcCore::Exit(0);
}
